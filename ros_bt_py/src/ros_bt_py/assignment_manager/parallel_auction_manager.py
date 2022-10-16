#  -------- BEGIN LICENSE BLOCK --------
#  Copyright 2022 FZI Forschungszentrum Informatik
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#     * Neither the name of the {copyright_holder} nor the names of its
#        contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#  -------- END LICENSE BLOCK --------
import threading
import uuid
from typing import Dict

import rospy
from rospy import ServiceException
from ros_bt_py_msgs.msg import AuctionMessage, RemoteCapabilitySlotStatus
from ros_bt_py_msgs.srv import (
    FindBestCapabilityExecutorRequest, FindBestCapabilityExecutorResponse, GetLocalBid,
    GetLocalBidResponse, GetLocalBidRequest, GetAvailableRemoteCapabilitySlots, ReserveRemoteCapabilitySlot,
    GetAvailableRemoteCapabilitySlotsRequest, GetAvailableRemoteCapabilitySlotsResponse,
    ReserveRemoteCapabilitySlotRequest,
)
from ros_bt_py.assignment_manager.assignment_manager import AssignmentManager

class AuctionStatus:
    def __init__(self, auction_id: str, auctioneer_id: str, deadline: rospy.Time):
        self.__auction_id = auction_id
        self.__auctioneer_id = auctioneer_id
        self.__deadline = deadline
        self.__is_closed = False
        self.__bids: Dict[str, Dict[str, float]] = {}
        self.__auction_status_lock = threading.Lock()

    @property
    def auction_id(self):
        return self.__auction_id

    @property
    def auctioneer_id(self):
        return self.__auctioneer_id

    @property
    def deadline(self):
        return self.__deadline

    @deadline.setter
    def deadline(self, new_deadline: rospy.Time):
        with self.__auction_status_lock:
            self.__deadline = new_deadline

    @property
    def is_closed(self):
        with self.__auction_status_lock:
            return self.__is_closed

    @is_closed.setter
    def is_closed(self, is_closed: bool):
        with self.__auction_status_lock:
            self.__is_closed = is_closed

    def get_valid_bids(self):
        with self.__auction_status_lock:
            return list(
                map(
                    lambda y: (y, self.__bids[y]["bid"]),
                    filter(lambda x: self.__bids[x]["executors"] > 0, self.__bids)
                    )
            )

    def get_bid(self, robot_name: str):
        with self.__auction_status_lock:
            try:
                rospy.logfatal(f"Get bid: {robot_name}")
                return self.__bids[robot_name]
            except KeyError as exc:
                rospy.logwarn(f"Failed to receive the bid for robot {robot_name}")
                raise exc

    def set_bid(self, robot_name: str, bid: float, no_executors: int, implementation_name: str):
        with self.__auction_status_lock:
            self.__bids[robot_name] = {
                "bid"           : bid,
                "executors"     : no_executors,
                "implementation": implementation_name
            }
            rospy.logfatal(f"Set bid: {robot_name} {self.__bids[robot_name]}")


class ParallelAuctionManager(AssignmentManager):

    running_auctions: Dict[str, AuctionStatus] = {}

    def __init__(
            self,
            local_topic_prefix: str,
            global_assignment_msg_topic_prefix: str,
    ):
        super().__init__(
            local_topic_prefix=local_topic_prefix,
            global_assignment_msg_topic_prefix=global_assignment_msg_topic_prefix)

        self.__local_topic_prefix = local_topic_prefix
        self.__auction_duration = rospy.Duration.from_sec(rospy.get_param("auction_duration_sec", 3.0))

        self.__global_auction_message_pub = rospy.Publisher(
            global_assignment_msg_topic_prefix,
            AuctionMessage,
            latch=False,
            queue_size=1
        )
        self.__global_auction_message_sub = rospy.Subscriber(
            global_assignment_msg_topic_prefix,
            AuctionMessage,
            self.global_auction_messages_callback,
            queue_size=1000
        )

        local_bid_service_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/mission_control/get_local_bid",
        )

        self.__local_bid_service = rospy.ServiceProxy(
            local_bid_service_topic,
            GetLocalBid
        )

        available_remote_slots_service_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/mission_control/get_available_remote_slots",
        )

        self.__available_remote_capability_slots_service = rospy.ServiceProxy(
            available_remote_slots_service_topic,
            GetAvailableRemoteCapabilitySlots
        )

        reserve_remote_slots_service_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/mission_control/reserve_remote_capability_slot",
        )

        self.__reserve_remote_capability_slot_service = rospy.ServiceProxy(
            reserve_remote_slots_service_topic,
            ReserveRemoteCapabilitySlot
        )
        self.running_auctions_lock = threading.RLock()

    def __get_available_local_remote_capability_slots(self) -> int:
        try:
            response: GetAvailableRemoteCapabilitySlotsResponse \
                = self.__available_remote_capability_slots_service.call(GetAvailableRemoteCapabilitySlotsRequest())
            return response.available_remote_capability_slots
        except ServiceException as exc:
            rospy.logerr(f"Failed to receive the available remote slots: {exc}")
            return 0

    def _is_running_auction_with_high_bid(self, auction_id: str, current_bid: int):
        return self._is_running_auction(auction_id) and \
               current_bid < self.running_auctions[auction_id].get_bid(self.__local_topic_prefix)["bid"]

    def _is_running_auction(self, auction_id: str):
        return not self.running_auctions[auction_id].is_closed

    def __handle_announcement_msg(self, msg: AuctionMessage):
        with self.running_auctions_lock:
            self.running_auctions[msg.auction_id] = AuctionStatus(
                auction_id=msg.auction_id,
                auctioneer_id=msg.sender_id,
                deadline=msg.deadline
            )

        bid_response: GetLocalBidResponse = self.__local_bid_service.call(
            GetLocalBidRequest(
                interface=msg.interface,
                inputs_topic=msg.inputs_topic,
                outputs_topic=msg.outputs_topic
            )
        )
        if not bid_response.success:
            rospy.logwarn(
                f"Failed to get local bid from: {self.__local_topic_prefix}, {bid_response.error_message}",
                logger_name="assignment_system"
            )
            return
        available_executors = self.__get_available_local_remote_capability_slots()

        with self.running_auctions_lock:
            self.running_auctions[msg.auction_id].set_bid(
                robot_name=self.__local_topic_prefix,
                bid=bid_response.bid,
                no_executors=available_executors,
                implementation_name=bid_response.implementation_name
            )

        with self.running_auctions_lock:
            running_auctions_by_bid = list(
                sorted(
                    filter(self._is_running_auction, self.running_auctions),
                    key=lambda x: self.running_auctions[x].get_bid(self.__local_topic_prefix)["bid"]
                )
            )

        possible_auctions = running_auctions_by_bid[:available_executors]
        impossible_auctions = running_auctions_by_bid[available_executors:]

        for auction_id in impossible_auctions:
            auction = self.running_auctions[auction_id]
            local_bid = auction.get_bid(self.__local_topic_prefix)

            bid_msg = AuctionMessage(
                auction_id=auction.auction_id,
                timestamp=rospy.Time.now(),
                sender_id=self.__local_topic_prefix,
                message_type=AuctionMessage.BID,
                bid=local_bid["bid"],
                number_of_executors=0,
                implementation_name=local_bid["implementation"]
            )
            self.__global_auction_message_pub.publish(bid_msg)

        for auction_id in possible_auctions:
            auction = self.running_auctions[auction_id]
            local_bid = auction.get_bid(self.__local_topic_prefix)

            bid_msg = AuctionMessage(
                auction_id=auction.auction_id,
                timestamp=rospy.Time.now(),
                sender_id=self.__local_topic_prefix,
                message_type=AuctionMessage.BID,
                bid=local_bid["bid"],
                number_of_executors=available_executors,
                implementation_name=local_bid["implementation"]
            )
            self.__global_auction_message_pub.publish(bid_msg)

        return

    def __handle_bid_msg(self, msg: AuctionMessage):
        try:
            with self.running_auctions_lock:
                auction_status = self.running_auctions[msg.auction_id]
        except KeyError:
            rospy.logerr("Unknown auction detected! Ignoring as not all required information is present!")
            return

        if auction_status.auctioneer_id != self.__local_topic_prefix:
            rospy.logdebug("Received bid for foreign auction, ignoring!")
            return

        if auction_status.is_closed or msg.timestamp > auction_status.deadline:
            rospy.logwarn(
                f"Ignoring bid from {msg.sender_id} for auction {msg.auction_id}, as it arrived after the deadline!"
                )
            return

        auction_status.set_bid(
            robot_name=msg.sender_id,
            bid=msg.bid,
            implementation_name=msg.implementation_name,
            no_executors=msg.number_of_executors
        )
        with self.running_auctions_lock:
            self.running_auctions[msg.auction_id] = auction_status

    def __handle_close_msg(self, msg: AuctionMessage):
        with self.running_auctions_lock:
            self.running_auctions[msg.auction_id].is_closed = True
        rospy.logdebug(f"Noting the close of auction: {msg.auction_id}")

    def __handle_abort_msg(self, msg: AuctionMessage):
        del self.running_auctions[msg.auction_id]

    def __handle_result_msg(self, msg: AuctionMessage):
        if msg.result_id == self.__local_topic_prefix:
            rospy.loginfo("Got awarded an auction!")

            available_slots = self.__get_available_local_remote_capability_slots()
            if available_slots >= 1:
                with self.running_auctions_lock:
                    for auction_id in filter(self._is_running_auction, self.running_auctions):
                        status = self.running_auctions[auction_id]
                        try:
                            local_bid = status.get_bid(self.__local_topic_prefix)
                        except KeyError:
                            continue

                        bid_msg = AuctionMessage(
                            auction_id=auction_id,
                            timestamp=rospy.Time.now(),
                            sender_id=self.__local_topic_prefix,
                            message_type=AuctionMessage.BID,
                            bid=local_bid["bid"],
                            number_of_executors=available_slots - 1,
                            implementation_name=local_bid["implementation"]
                        )
                        self.__global_auction_message_pub.publish(bid_msg)

            self.__reserve_remote_capability_slot_service.call(
                ReserveRemoteCapabilitySlotRequest(
                    remote_mission_control=msg.sender_id,
                    implementation_name=
                    self.running_auctions[msg.auction_id].get_bid(self.__local_topic_prefix)["implementation"]
                )
            )
            return
        rospy.logdebug(f"Noting the result announcement for auction: {msg.auction_id}")

    def __handle_update_msg(self, msg: AuctionMessage):
        pass

    def global_auction_messages_callback(self, msg: AuctionMessage):
        if msg.sender_id == self.__local_topic_prefix:
            rospy.logdebug_throttle(1, f"Ignoring local message: {msg.sender_id}")
            return

        if msg.message_type == msg.ANNOUNCEMENT:
            rospy.logdebug(f"Handling announcement auction message {msg.auction_id}")
            self.__handle_announcement_msg(msg)
            return
        if msg.message_type == msg.BID:
            rospy.logdebug(f"Handling bid auction message {msg.auction_id}")
            self.__handle_bid_msg(msg)
            return
        if msg.message_type == msg.CLOSE:
            rospy.logdebug(f"Handling close auction message {msg.auction_id}")
            self.__handle_close_msg(msg)
            return
        if msg.message_type == msg.ABORT:
            rospy.logdebug(f"Handling abort auction message {msg.auction_id}")
            self.__handle_abort_msg(msg)
            return
        if msg.message_type == msg.RESULT:
            rospy.logdebug(f"Handling result auction message {msg.auction_id}")
            self.__handle_result_msg(msg)
            return
        if msg.message_type == msg.UPDATE:
            rospy.logdebug(f"Handling update auction message {msg.auction_id}")
            self.__handle_update_msg(msg)
            return

    def find_best_capability_executor(
            self,
            goal: FindBestCapabilityExecutorRequest
    ) -> FindBestCapabilityExecutorResponse:
        response = FindBestCapabilityExecutorResponse()

        current_auction_id = str(uuid.uuid4())
        current_deadline = rospy.Time.now() + self.__auction_duration

        with self.running_auctions_lock:
            self.running_auctions[current_auction_id] = AuctionStatus(
                auction_id=current_auction_id,
                auctioneer_id=self.__local_topic_prefix,
                deadline=current_deadline
            )

        self.__global_auction_message_pub.publish(
            AuctionMessage(
                auction_id=current_auction_id,
                timestamp=rospy.Time.now(),
                sender_id=self.__local_topic_prefix,
                message_type=AuctionMessage.ANNOUNCEMENT,
                interface=goal.capability,
                inputs_topic=goal.inputs_topic,
                outputs_topic=goal.outputs_topic,
                deadline=current_deadline
            )
        )

        rospy.sleep(self.__auction_duration + rospy.Duration.from_sec(0.02))

        self.__global_auction_message_pub.publish(
            AuctionMessage(
                auction_id=current_auction_id,
                timestamp=rospy.Time.now(),
                sender_id=self.__local_topic_prefix,
                message_type=AuctionMessage.CLOSE,
            )
        )
        with self.running_auctions_lock:
            bids = self.running_auctions[current_auction_id].get_valid_bids()

        rospy.logfatal(f"Auction status: {self.running_auctions[current_auction_id]}")

        if len(bids) < 1:
            response.success = False
            response.error_message = "No valid bids received! Auction has failed!"
            rospy.logerr(response.error_message)
            return response

        sorted_bids = sorted(bids, key=lambda x: x[1])
        top_bid = sorted_bids[0]
        top_bid_id = top_bid[0]

        rospy.loginfo(f"Top bid for auction {current_auction_id} - {top_bid_id}!")

        top_bid_info = self.running_auctions[current_auction_id].get_bid(top_bid_id)

        response.success = True
        response.implementation_name = top_bid_info["implementation"]
        response.execute_local = (top_bid_id == self.__local_topic_prefix)
        response.executor_mission_control_topic = top_bid_id

        # TODO: Realize the only one bid case via a ROS parameter!
        if len(sorted_bids) > 1:
            response.max_allowed_costs = sorted_bids[1][1]
        else:
            response.max_allowed_costs = top_bid[1] * 1.2

        self.__global_auction_message_pub.publish(
            AuctionMessage(
                auction_id=current_auction_id,
                timestamp=rospy.Time.now(),
                sender_id=self.__local_topic_prefix,
                message_type=AuctionMessage.RESULT,
                result_id=top_bid_id,
                implementation_name=top_bid_info["implementation"]
            )
        )
        return response
