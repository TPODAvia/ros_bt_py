import jsonpickle

from ros_bt_py_msgs.srv import GetJsonpickleInstance, GetJsonpickleInstanceRequest, GetJsonpickleInstanceResponse


def get_jsonpickle_instance(request):
    res = GetJsonpickleInstanceResponse()
    res.success = True

    try:
        requested_type = jsonpickle.decode(requrest.serialized_type)
    except ValueError as e:
        res.success = False
        res.error_message = str(e)
        return res

    try:
        instance = requested_type()
    except AttributeError as e:
        res.success = False
        res.error_message = ('Unable to construct instance from given type: ' +
                             str(e))
        return res

    res.serialized_instance = jsonpickle.encode(instance)
    if res.serialized_instance is None:
        res.success = False
        res.error_message = 'Unable to jsonpickle instance {}'.format(repr(instance))
    return res
