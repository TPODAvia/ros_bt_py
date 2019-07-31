
// import { Component } from 'preact';
// import { render } from 'preact';
// import { createRef } from 'preact';
var Component = React.Component;
var render = ReactDOM.render;
var createRef = React.createRef;
var Fragment = React.Fragment; //'x-fragment';

// uuid is used to assign unique IDs to tags so we can use labels properly
let idx = 0;
const uuid = () => idx++;

// and another one for errors
let error_idx = 0;
const error_id = () => error_idx++;

function typesCompatible(a, b)
{
  if (a.nodeName === b.nodeName) {
    return false;
  }

  if (a.kind === b.kind) {
    return false;
  }

  var from = a.kind === 'output' ? a : b;
  var to = a.kind === 'input' ? a : b;

  // object is compatible with anything
  if (to.type === "{\"py/type\": \"__builtin__.object\"}" || to.type === "{\"py/type\": \"builtins.object\"}") {
    return true;
  }

  return prettyprint_type(from.type) === prettyprint_type(to.type);
}

var python_builtin_types = [
  'int',
  'float',
  'long',
  'str',
  'basestring',
  'unicode',
  'bool',
  'list',
  'dict',
  'set',
  'type'
];

function prettyprint_type(jsonpickled_type) {
  var json_type = JSON.parse(jsonpickled_type);
  if (json_type['py/type'] !== undefined)
  {
    // Remove the "builtin" prefix jsonpickle adds
    return json_type['py/type']
      .replace('__builtin__.', '')
      .replace('builtins.', '')
      .replace(/^basestring$/, 'string')
      .replace(/^unicode$/, 'string')
      .replace(/^str$/, 'string');
  }

  // If the type doesn't have a py/type field, maybe it's an
  // OptionRef?
  if (json_type['py/object'] !== undefined &&
      json_type['py/object'] === 'ros_bt_py.node_config.OptionRef')
  {
    return 'OptionRef(' + json_type['option_key'] + ')';
  }

  if (json_type['py/reduce'] !== undefined &&
      json_type['py/reduce'][0] !== undefined &&
      json_type['py/reduce'][0]['py/type'] !== undefined &&
      json_type['py/reduce'][0]['py/type'] === 'collections.OrderedDict')
  {
    return json_type['py/reduce'][0]['py/type'];
  }

  return 'Unknown type object: ' + jsonpickled_type;
}

function getDefaultValue(typeName, options)
{
  if (typeName === 'type')
  {
    return {type: 'type',
            value: 'int'};
  }
  else if (typeName === 'int' || typeName === 'long')
  {
    return {type: 'int',
            value: 0};
  }
  else if (typeName === 'str' || typeName === 'basestring' || typeName === 'unicode'
           || typeName === 'string')
  {
    return {type: 'string',
            value: 'foo'};
  }
  else if (typeName === 'float')
  {
    return {type: 'float',
            value: 1.2};
  }
  else if (typeName === 'bool')
  {
    return {type: 'bool',
            value: true};
  }
  else if (typeName === 'list')
  {
    return {type: 'list',
            value: []};
  }
  else if (typeName === 'dict')
  {
    return {type: 'dict',
            value: {}};
  }
  else if (typeName.startsWith('OptionRef('))
  {
    var optionTypeName = typeName.substring(
        'OptionRef('.length, typeName.length - 1);
    var optionType = options.find(x => {
      return x.key === optionTypeName;
    });
    if (optionType)
    {
      return getDefaultValue(
        prettyprint_type(optionType.serialized_value));
    }
    else
    {
      return {
        type: 'unset_optionref',
        value: 'Ref to "' + optionTypeName + '"'
      };
    }
  }
  else if (typeName === 'collections.OrderedDict')
  {
    return {type: 'collections.OrderedDict',
            value: {"py/reduce": [{"py/type": "collections.OrderedDict"}, {"py/tuple": [[]]}, null, null, null]}
    };
  }
  else
  {
    return {type: '__' + typeName,
            value: {}};
  }
}

// Get the distance between two sets of coordinates (expected to be
// arrays with 2 elements each)
function getDist(a, b)
{
  return Math.sqrt(Math.pow(a[0] - b[0], 2) + Math.pow(a[1] - b[1], 2));
}

function treeIsEditable(tree_msg)
{
  return tree_msg.state === "EDITABLE";
}

function selectIOGripper(vertex_selection, data)
{
  return vertex_selection
    .selectAll("." +
               data.data_kind.substring(0, data.data_kind.length - 1) +
               "-gripper-group")
    .filter(d => d.nodeName === data.node_name)
    .filter(d => d.key === data.data_key);
}

function getMessageType(str)
{
  var message_type = str;
  var msg_string = ".msg._";
  var first_index = message_type.indexOf(msg_string);
  var service = false;
  if (first_index == -1)
  {
    first_index = message_type.indexOf(".srv._");
    service = true;
  }
  var package_name = message_type.substr(0,first_index);
  var second_index = message_type.indexOf(".", first_index+msg_string.length);
  var message_name = message_type.substr(second_index+1);
  message_type = package_name+"/"+message_name;
  return {message_type: message_type, service: service};
}

function getShortDoc(doc) {
  if (!doc || doc == null || doc.length == 0)
  {
    return "No documentation provided";
  } else {
    var index = doc.indexOf("**Behavior Tree I/O keys**");
    if (index < 0)
    {
      return doc;
    } else {
      return doc.substring(0, index).trim();
    }
  }
}

class NodeListItem extends Component {
  constructor(props)
  {
    super(props);
    this.state = {
      collapsed: this.props.collapsed,
    };
  }

  renderIOTable(nodedata_list, title) {
    // If there are no items in the list, don't generate any DOM
    // elements.
    if (nodedata_list.length == 0)
    {
      return null;
    }
    var rows = nodedata_list.map(data => {
      return (
        <tr key={title+data.key}>
          <td title={data.key} className="io_key text-truncate maxw0">{data.key}</td>
          <td title={prettyprint_type(data.serialized_value)}className="io_type text-truncate maxw0 text-muted pl-2">
            {prettyprint_type(data.serialized_value)}
          </td>
        </tr>);
    });

    return (
      <div className="io_values list-group-item">
        <h5>{title}</h5>
        <table className="table"><tbody>
            {rows}
        </tbody></table>
      </div>
    );
  };

  onClick(e, node) {
    this.props.onSelectionChange(this.props.node);
  }

  toggleCollapsed(event) {
    this.setState({collapsed: !this.state.collapsed});
    event.stopPropagation();
  }

  render() {
    var collapsible_icon = "cursor-pointer fas fa-angle-up";
    if (this.state.collapsed)
    {
      collapsible_icon = "cursor-pointer fas fa-angle-down";
    }
    var io_table = null;
    if (!this.state.collapsed)
    {
      io_table = (
        <div className="list-group">
          { this.renderIOTable(this.props.node.options, 'Options') }
          { this.renderIOTable(this.props.node.inputs, 'Inputs') }
          { this.renderIOTable(this.props.node.outputs, 'Outputs') }
        </div>
      );
    }
    var node_type = null;
    if (this.props.node.max_children < 0)
    {
      node_type = "Flow control";
    } else if (this.props.node.max_children > 0)
    {
      node_type = "Decorator";
    } else {
      node_type = "Leaf";
    }

    return (
      <div className="border rounded p-2 mb-2"
           onClick={this.onClick.bind(this)}>
        <div className="d-flex justify-content-between">
          <div className="d-flex minw0">
            <h4 title={this.props.node.node_class} className="node_class text-truncate">{this.props.node.node_class}</h4>
            <i title={getShortDoc(this.props.node.doc)} class="fas fa-question-circle pl-2 pr-2"></i>
          </div>
          <div className="d-flex minw0">
            <i onClick={this.toggleCollapsed.bind(this)} class={collapsible_icon}></i>
          </div>
        </div>
        <h5 title={this.props.node.module} className="node_module text-truncate text-muted">{this.props.node.module}</h5>
        <div>{
          node_type + ' (max_children: ' + (this.props.node.max_children >= 0 ? this.props.node.max_children : '∞') + ')'}</div>
        {io_table}
      </div>
    );
  };
}

class NodeList extends Component
{
  constructor(props)
  {
    super(props);

    this.state = {
      package_name: 'ros_bt_py.nodes.sequence',
      package_loader_collapsed: true,
    };

    this.handleChange = this.handleChange.bind(this);
  }

  componentDidMount()
  {
    this.props.getNodes('');
  }


  handleChange(e)
  {
    this.setState({package_name: e.target.value});
  }

  toggleCollapsed(event) {
    this.setState({package_loader_collapsed: !this.state.package_loader_collapsed});
    event.stopPropagation();
  }

  render()
  {
    var byName = function(a, b) {
      if (a.node_class < b.node_class)
      {
        return -1;
      }
      else if (a.node_class > b.node_class)
      {
        return 1;
      }

      return 0;
    };

    var moduleThenName = function(a, b) {
      if (a.module < b.module)
      {
        return -1;
      }
      else if (a.module > b.module)
      {
        return 1;
      }

      return byName(a, b);
    };

    var items = this.props.availableNodes
        .sort(byName)
//        .sort(moduleThenName)
        .map( (node) => {
          return (<NodeListItem node={node}
                           key={node.module + node.node_class}
                           collapsed={true}
                           onSelectionChange={this.props.onSelectionChange}/>);
    });

    var collapsible_icon = "fas fa-angle-up";
    var package_loader = null;
    if (this.state.package_loader_collapsed)
    {
      collapsible_icon = "fas fa-angle-down";
    } else {
      package_loader = (
        <div className="form-group">
          <button id="refresh"
                  className="btn btn-block btn-primary mt-2"
                  onClick={() => this.props.getNodes('')}>
            Refresh
          </button>
          <input type="text" id="package_name"
                 className="form-control mt-2"
                 value={this.state.package_name}
                 onChange={this.handleChange}/>
          <button id="load_package"
                  className="btn btn-block btn-primary mt-2"
                  onClick={() => this.props.getNodes(this.state.package_name)}>
            Load package
          </button>
        </div>
      );
    }

    return(
      <div className="available-nodes m-1">
        <div className="border rounded mb-2">
          <div onClick={this.toggleCollapsed.bind(this)} className="text-center cursor-pointer font-weight-bold m-2">Package Loader <i class={collapsible_icon}></i></div>
          {package_loader}
        </div>
        <div className="vertical_list">
          {items}
        </div>
      </div>
    );
  }
}


class ErrorHistory extends Component {
  constructor(props)
  {
    super(props);
  }

  render()
  {
    var buttons = null;
    var sorting_icon = (<span aria-hidden="true" className="fas fa-sort-up" />);
    if (this.props.sorting_asc)
    {
      sorting_icon = (<span aria-hidden="true" className="fas fa-sort-down" />);
    }
    if (this.props.history && this.props.history.length > 0)
    {
      buttons = (<div className="clear-error">
                  <button className="btn btn-primary m-1" onClick={() => this.props.changeSorting(!this.props.sorting_asc)}>
                    {sorting_icon}
                  </button>
                  <button className="btn btn-primary m-1" onClick={this.props.clearErrors}>Clear errors</button>
                 </div>);
    }
    var error_history = this.props.history.slice(0);
    if (!this.props.sorting_asc)
    {
      error_history.reverse();
    }
    return (
      <ul className="w-100 list-group">
        { buttons }
        { error_history.map((errorEntry) => {
          return (
            <li className="list-group-item" key={errorEntry.id}>
              <p>
                <small className="message-date">{(new Date(errorEntry.time)).toTimeString()}</small>
                <br />
                <span>{errorEntry.text}</span>
              </p>
            </li>
          );
        })
        }
      </ul>
    );
  }
}

function ExecutionBar(props)
{
  return (
    <header id="header" className="d-flex flex-column flex-md-row align-items-center control-bar">
      <NamespaceSelect
        ros={props.ros}
        connected={props.connected}
        currentNamespace={props.currentNamespace}
        onNamespaceChange={props.onNamespaceChange}
        onError={props.onError}/>
      <DebugControls
        ros={props.ros}
        bt_namespace={props.currentNamespace}
        onError={props.onError}
        onPublishingSubtreesChange={props.onPublishingSubtreesChange}/>
      <TickControls
        ros={props.ros}
        bt_namespace={props.currentNamespace}
        onError={props.onError}/>
      <Spacer/>
      <LoadSaveControls
        ros={props.ros}
        bt_namespace={props.currentNamespace}
        tree_message={props.tree_message}
        onError={props.onError}/>
    </header>
  );
}

class Spacer extends Component
{
  constructor(props)
  {
    super(props);
  }

  componentDidMount()
  {
  }

  render()
  {
    return (
      <Fragment>
        <div className="spacer">
        </div>
      </Fragment>
    );
  }
}

class NamespaceSelect extends Component
{
  constructor(props)
  {
    super(props);

    this.state = {
      available_namespaces: [],
      edit: false,
      ros_uri: props.ros.socket.url
    };

    this.updateAvailableNamespaces = this.updateAvailableNamespaces.bind(this);
    this.changeRosbridgeServer = this.changeRosbridgeServer.bind(this);
    this.editRosbridgeServer = this.editRosbridgeServer.bind(this);
    this.saveRosbridgeServer = this.saveRosbridgeServer.bind(this);
    this.handleNamespaceChange = this.handleNamespaceChange.bind(this);
  }

  componentDidMount()
  {
    this.servicesForTypeClient = new ROSLIB.Service({
      ros: this.props.ros,
      name: '/rosapi/services_for_type',
      serviceType: 'rosapi/ServicesForType'
    });

    this.updateAvailableNamespaces();
  }

  updateAvailableNamespaces()
  {
    this.servicesForTypeClient.callService(
      // Search for all Tree topics - we expect each BT node to
      // publish one of these, and also offer the corresponding
      // editing and runtime control services.
      new ROSLIB.ServiceRequest({
        type: 'ros_bt_py_msgs/AddNode'
      }),
      function(response) {
        var namespaces = response.services.map(
          // Chop off the topic name (but not the last slash), which leaves us with the BT
          // namespace
          x => x.substr(0, x.lastIndexOf('/')) + '/'
        );
        this.setState({
          available_namespaces: namespaces
        });
        if (this.props.currentNamespace === '' && namespaces.length > 0)
        {
          this.props.onNamespaceChange(namespaces[0]);
        }
      }.bind(this));
  }

  changeRosbridgeServer(event)
  {
    this.setState({ros_uri: event.target.value});
  }

  editRosbridgeServer()
  {
    if (this.state.edit)
    {
      this.setState({edit: false});
    } else {
      console.log("edit rosbridge server");
      this.setState({edit: true});
    }
  }

  saveRosbridgeServer()
  {
    console.log("save rosbridge server ", );

    this.setState({edit: false});

    var old_uri = window.location.toString();
    var new_uri = old_uri;
    if (window.location.search.length > 0) {
      new_uri = old_uri.replace(window.location.search, "?ros_uri="+this.state.ros_uri)
    } else {
      new_uri = old_uri + "?ros_uri="+this.state.ros_uri;
    }
    if (old_uri != new_uri)
    {
      window.location.assign(new_uri);
    }
  }

  handleNamespaceChange(event)
  {
    this.props.onNamespaceChange(event.target.value);
  }

  render()
  {
    var edit = null;
    if (this.state.edit) {
      edit = (<div className="form-inline">
                <label className="ml-1">Rosbridge Server:
                  <input type="text" value={this.state.ros_uri} onChange={this.changeRosbridgeServer}/>
                  <button onClick={this.saveRosbridgeServer.bind(this)}
                          className="btn btn-primary m-1">
                    Save
                  </button>
                </label>
              </div>);
    }
    var connected_class = "fas fa-wifi connected";
    var connected_title = "Connected";
    if (!this.props.connected) {
      connected_class = "fas fa-wifi disconnected";
      connected_title = "Disconnected";
    }
    return (
      <Fragment>
        <span aria-hidden="true" title={connected_title} className={connected_class} />
        <div className="form-inline">
          <label className="ml-1">BT Namespace:
            <select className="custom-select ml-1"
                    value={this.props.currentNamespace}
                    onChange={this.handleNamespaceChange}>
              {
                this.state.available_namespaces.map(
                  x => (<option key={x}
                                value={x}>{x}</option>))
              }
            </select>
          </label>
        </div>
        <button type="button"
                className="btn btn-sm m-1"
                onClick={this.updateAvailableNamespaces}>
          <span aria-hidden="true" className="fas fa-sync" />
          <span className="sr-only">Refresh Namespaces</span>
        </button>
        <button type="button"
                className="btn btn-sm m-1"
                onClick={this.editRosbridgeServer}>
          <span aria-hidden="true" className="fas fa-cog" />
          <span className="sr-only">Edit rosbridge server</span>
        </button>
        {edit}
      </Fragment>
    );
  }
}

class SelectTree extends Component
{
  constructor(props)
  {
    super(props);

    this.onChange = this.onChange.bind(this);
  }

  onChange(event)
  {
    var value = parseInt(event.target.value);
    if (value < 0)
    {
      this.props.onSelectedTreeChange(
        /*is_subtree=*/ false,
        /*name=*/ ''); // no name needed, there's only one not-subtree
    }
    else
    {
      this.props.onSelectedTreeChange(
        /*is_subtree=*/ true,
        /*name=*/ this.props.subtreeNames[value]);
    }
  }

  render()
  {
    var selected = "main";
    if (this.props.selected_tree.is_subtree && this.props.selected_tree.name !== "")
    {
      selected = this.props.selected_tree.name;
    }
    return (
      <div>
        <label className="form-inline m-1">Tree:
          <select className="custom-select ml-1"
                  value={this.props.subtreeNames.indexOf(selected)}
                  onChange={this.onChange}>
            <option value="-1">Main Tree</option>
            <optgroup label="Subtrees">
              {
                this.props.subtreeNames.map(
                  (name, index) =>  (<option key={name} value={index}>{name}</option>))
              }
            </optgroup>
          </select>
        </label>
      </div>
    );
  }
}

class SelectEditorSkin extends Component
{
  constructor(props)
  {
    super(props);

    this.onChange = this.onChange.bind(this);
  }

  onChange(event)
  {
    console.log("changing editor skin to " + event.target.value);
    this.props.changeSkin(event.target.value);
  }

  render()
  {
    return (
      <div>
        <label className="form-inline m-1">Color scheme:
          <select className="custom-select ml-1"
                  defaultValue="darkmode"
                  onChange={this.onChange}>
            <option value="darkmode">Dark Mode</option>
            <option value="light">Light Mode</option>
          </select>
        </label>
      </div>
    );
  }
}
class TickControls extends Component
{
  constructor(props)
  {
    super(props);

  }
  componentDidMount()
  {
    this.tick_service = new ROSLIB.Service({
      ros: this.props.ros,
      name: this.props.bt_namespace + 'control_tree_execution',
      serviceType: 'ros_bt_py_msgs/ControlTreeExecution'
    });

  }
  controlExec(command)
  {
    this.tick_service.callService(
      new ROSLIB.ServiceRequest({
        // TICK_ONCE = 1
        // TICK_PERIODICALLY = 2
        // TICK_UNTIL_RESULT = 3
        // STOP = 4
        // RESET = 5
        // SHUTDOWN = 6
        // SETUP_AND_SHUTDOWN = 7
        command: command
      }),
      function(response) {
        if (response.success) {
          console.log('called ControlTreeExecution service successfully');
        }
        else {
          this.props.onError(response.error_message);
        }
      }.bind(this));
  }

  render()
  {
    return (
      <Fragment>
        <button onClick={this.controlExec.bind(this, 1)}
                className="btn btn-primary m-1">
          Tick Once
        </button>
        <button onClick={this.controlExec.bind(this, 2)}
                className="btn btn-primary m-1">
          Tick Periodically
        </button>
        <button onClick={this.controlExec.bind(this, 3)}
                className="btn btn-primary m-1">
          Tick Until Result
        </button>
        <button onClick={this.controlExec.bind(this, 4)}
                className="btn btn-primary m-1">
          Stop
        </button>
        <button onClick={this.controlExec.bind(this, 5)}
                className="btn btn-primary m-1">
          Reset
        </button>
        <button onClick={this.controlExec.bind(this, 6)}
                className="btn btn-primary m-1">
          Shutdown
        </button>
      </Fragment>
    );
  }
}

class LoadSaveControls extends Component
{
  constructor(props)
  {
    super(props);
    this.fileref = React.createRef()
    this.fileReader = new FileReader();
  }

  componentDidMount()
  {
    this.load_service = new ROSLIB.Service({
      ros: this.props.ros,
      name: this.props.bt_namespace + 'load_tree',
      serviceType: 'ros_bt_py_msgs/LoadTree'
    });

    this.clear_service = new ROSLIB.Service({
      ros: this.props.ros,
      name: this.props.bt_namespace + 'clear',
      serviceType: 'ros_bt_py_msgs/ClearTree'
    });

  }

  openFileDialog()
  {
    this.fileref.current.click();
  }

  handleFileRead(event)
  {
    var msgs = jsyaml.loadAll(this.fileReader.result);

    var msg = null;
    for (var i = 0; i < msgs.length; i++) {
      if (msgs[i] != null) {
        msg = msgs[i];
      }
    }

    this.load_service.callService(
      new ROSLIB.ServiceRequest({
        tree: msg
      }),
      function(response) {
        if (response.success) {
          console.log('called LoadTree service successfully');
        }
        else {
          this.props.onError(response.error_message);
        }
      }.bind(this),
      function(failed) {
        this.props.onError('Error loading tree, is your yaml file correct? ' + failed)
      }.bind(this));
  }

  downloadURI(uri, name) {
    var link = document.createElement("a");
    link.download = name;
    link.href = uri;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  }

  newTree()
  {
    if (window.confirm("Do you want to create a new tree? Warning: This will discard the tree that is loaded at the moment."))
    {
      this.clear_service.callService(
        new ROSLIB.ServiceRequest({}),
        function(response) {
          if (response.success) {
            console.log('called ClearTree service successfully');
          }
          else {
            this.props.onError(response.error_message);
          }
        }.bind(this),
        function(failed) {
          this.props.onError('Error clearing tree ' + failed)
        }.bind(this));
    }
  }

  loadTree(event)
  {
    this.fileReader.onloadend = this.handleFileRead.bind(this);
    this.fileReader.readAsText(event.target.files[0]);
  }

  saveTree()
  {
    var msg = jsyaml.safeDump(this.props.tree_message);
    this.downloadURI('data:text/plain,'+encodeURIComponent(msg), "tree.yaml");
  }

  render()
  {
    return (
      <Fragment>
        <button onClick={this.newTree.bind(this)}
                className="btn btn-primary m-1">
          New
        </button>
        <div>
          <input ref={this.fileref} type="file" style={{display:"none"}} onChange={this.loadTree.bind(this)}/>
          <button onClick={this.openFileDialog.bind(this)}
                  className="btn btn-primary m-1">
            Load
          </button>
        </div>
        <button onClick={this.saveTree.bind(this)}
                className="btn btn-primary m-1">
          Save
        </button>
      </Fragment>
    );
  }
}

class DebugControls extends Component
{
  constructor(props)
  {
    super(props);

    this.state = {
      debugging: false,
      publishing_subtrees: false
    };

    this.debug_settings_sub = new ROSLIB.Topic({
      ros: props.ros,
      name: props.bt_namespace + 'debug/debug_settings',
      messageType: 'ros_bt_py_msgs/DebugSettings'
    });

    this.set_execution_mode_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'debug/set_execution_mode',
      serviceType: 'ros_bt_py_msgs/SetExecutionMode'
    });

    this.step_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'debug/continue',
      serviceType: 'ros_bt_py_msgs/Continue'
    });

    this.debugCheckID = 'debug_' + uuid();
    this.publishSubtreesID = 'publish_subtrees_' + uuid();

    this.onNewDebugSettings = this.onNewDebugSettings.bind(this);
    this.onClickStep = this.onClickStep.bind(this);
    this.handleDebugChange = this.handleDebugChange.bind(this);
    this.handlePubSubtreesChange = this.handlePubSubtreesChange.bind(this);
  }

  componentDidMount()
  {
    this.debug_settings_sub.subscribe(this.onNewDebugSettings);
  }

  componentWillUnmount()
  {
    this.debug_settings_sub.unsubscribe(this.onNewDebugSettings);
  }

  onNewDebugSettings(msg)
  {
    this.setState({debugging: msg.single_step,
                   publishing_subtrees: msg.publish_subtrees});
  }

  onClickStep()
  {
    this.step_service.callService(
      new ROSLIB.ServiceRequest({}),
      function(response) {
        if (response.success) {
          console.log('stepped successfully');
        }
        else {
          this.props.onError(response.error_message);
        }
      }.bind(this));
  }

  handleDebugChange(event)
  {
    var enable = event.target.checked;
    this.set_execution_mode_service.callService(
      new ROSLIB.ServiceRequest({
        single_step: enable,
        publish_subtrees: this.state.publishing_subtrees,
        collect_performance_data: true
      }),
      function(response) {
        if (enable) {
          console.log('enabled stepping');
        }
        else {
          console.log('disabled stepping');
        }
      }.bind(this));
    this.setState({debugging: enable});
  }

  handlePubSubtreesChange(event)
  {
    var enable = event.target.checked;
    this.props.onPublishingSubtreesChange(enable);
    this.set_execution_mode_service.callService(
      new ROSLIB.ServiceRequest({
        single_step: this.state.debugging,
        publish_subtrees: enable,
        collect_performance_data: true
      }),
      function(response) {
        if (enable) {
          console.log('enabled stepping');
        }
        else {
          console.log('disabled stepping');
        }
      }.bind(this));
    this.setState({publishing_subtrees: enable});
  }

  render()
  {
    return (
      <Fragment>
        <div className="custom-control custom-checkbox m-1">
          <input type="checkbox"
                 id={this.debugCheckID}
                 className="custom-control-input"
                 checked={this.state.debugging}
                 onChange={this.handleDebugChange} />
          <label className="custom-control-label"
                 htmlFor={this.debugCheckID}>Debug</label>
        </div>
        <div className="custom-control custom-checkbox m-1">
          <input type="checkbox"
                 id={this.publishSubtreesID}
                 className="custom-control-input"
                 checked={this.state.publishing_subtrees}
                 onChange={this.handlePubSubtreesChange} />
          <label className="custom-control-label"
                 htmlFor={this.publishSubtreesID}>Publish Subtrees</label>
        </div>
        <button onClick={this.onClickStep}
                className="btn btn-primary m-1">
          Step
        </button>
      </Fragment>
    );
  }
}

class D3BehaviorTreeEditor extends Component
{
  constructor(props)
  {
    super(props);

    this.spacing = 80;

    this.min_node_drag_distance=15;

    this.io_gripper_spacing = 10;
    this.io_gripper_size = 15;
    this.max_io_gripper_size = 15;

    this.nextWiringSource = null;
    this.nextWiringTarget = null;

    this.draggedNode = null;
    this.dragging = false;

    // ### Pan and zoom stuff ###
    this.zoomObject = null;
    // Begin panning if the mouse is less than this away from the
    // viewport's edge
    this.dragPanBoundary = 50;
    this.panIntervalID = null;
    this.panDirection = [0.0, 0.0];
    this.panRate = 30;
    this.panPerFrame = 10.0;

    this.wire_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'wire_data',
      serviceType: 'ros_bt_py_msgs/WireNodeData'
    });

    this.unwire_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'unwire_data',
      serviceType: 'ros_bt_py_msgs/UnwireNodeData'
    });

    this.move_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'move_node',
      serviceType: 'ros_bt_py_msgs/MoveNode'
    });

    this.replace_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'replace_node',
      serviceType: 'ros_bt_py_msgs/ReplaceNode'
    });

    this.svg_ref = createRef();
    this.viewport_ref = createRef();

    this.dragPanTimerHandler = this.dragPanTimerHandler.bind(this);
    this.resetView = this.resetView.bind(this);
    this.getIOCoords = this.getIOCoords.bind(this);
    this.getIOCoordsFromNode = this.getIOCoordsFromNode.bind(this);
  }

  componentDidMount()
  {
    // Give Viewport pan / zoom
    var viewport = d3.select(this.viewport_ref.current);
    var width = viewport.node().getBoundingClientRect().width;
    var height = viewport.node().getBoundingClientRect().height;

    this.zoomObject = d3.zoom();
    var container = d3.select(this.svg_ref.current);

    viewport
      .call(this.zoomObject.scaleExtent([0.3, 1.0]).on("zoom", function () {
        container.attr("transform", d3.event.transform);
      }))
      .call(this.zoomObject.translateTo, 0.0, (height * 0.5) - 10.0);

    // Add Mousemove listener to pan viewport while draggins
    viewport.on("mousemove.pan_if_drag", this.canvasMousemovePanHandler.bind(this));
    viewport
      .on("click", () => {
        // Deselect any selected node if the user clicks on the background
        this.props.onSelectionChange(null);
        this.props.onSelectedEdgeChange(null);
      });
  }

  render()
  {
    var editor_classes = "reactive-svg " + this.props.skin;
    return (
      <svg id="editor_viewport"
           ref={this.viewport_ref}
           className={editor_classes}>
        <g id="container" ref={this.svg_ref}>
          { // order is important here - SVG draws things in the order
            // they appear in the markup!
          }
          <g className="edges"/>
          <g className="vertices"/>
          { // Data Graph should be above the node graph
            // (since it can be toggled on and off)
          }
          <g className="data_graph">
            { // We want the edges below the vertices here because that looks nicer
            }
            <g className="data_edges" />
            <g className="data_vertices" />
          </g>
          { // This is for the targets that appear when the user is dragging
            // a node to reposition it.
            // Obviously, these should be above anything else!
          }
          <g className="drop_targets"
             visibility="hidden" />
        </g>
          <text x="10"
                y="20"
                fill="#FFFFFF"
                text-anchor="left"
                alignment-baseline="top"
                className="cursor-pointer svg-button"
                onClick={this.resetView}>
            Reset View
          </text>

      </svg>
    );
  }

  componentDidUpdate(prevProps, prevState)
  {
    if (this.props.tree_message !== prevProps.tree_message)
    {
      this.drawEverything(this.props.tree_message);

      // Disable all interaction (except for zooming and panning) when
      // the tree isn't editable
      if (treeIsEditable(this.props.tree_message))
      {

      }
    }
    // Hide or show data graph
    if (this.props.showDataGraph)
    {
      d3.select(this.svg_ref.current).select(".data_graph").attr("visibility", "visible");
    }
    else
    {
      d3.select(this.svg_ref.current).select(".data_graph").attr("visibility", "hidden");
    }

    if (this.props.selectedNodeName)
    {
      var that = this;
      d3.select(this.svg_ref.current)
        .selectAll(".btnode")
        .each(function(d) {
          d3.select(this).classed("node-selected", (d.id === that.props.selectedNodeName));
        });
    }
    else
    {
      d3.select(this.svg_ref.current)
        .selectAll(".btnode")
        .each(function(d) {
          d3.select(this).classed("node-selected", false);
        });

    }
  }

  drawEverything(tree_msg)
  {
    if (!tree_msg)
    {
      return;
    };

    var onlyKeyAndType = nodeData => ({
      key: nodeData.key,
      serialized_type: nodeData.serialized_type
    });
    // Trim the serialized data values from the node data - we won't
    // render them, so don't clutter the DOM with the data
    var trimmed_nodes = tree_msg.nodes.map(function(node) {
      return {
        node_class: node.node_class,
        module: node.module,
        name: node.name,
        state: node.state,
        max_children: node.max_children,
        child_names: node.child_names,
        options: node.options.map(onlyKeyAndType),
        inputs: node.inputs.map(onlyKeyAndType),
        outputs: node.outputs.map(onlyKeyAndType)
      };
    });

    var forest_root = {
      name: "__forest_root",
      child_names: [],
      inputs: [],
      outputs: [],
      options: []
    };

    if (trimmed_nodes.findIndex(x => x.name === "__forest_root") < 0)
    {
      trimmed_nodes.push(forest_root);
    }
    // Update the visual tree
    var parents = {};
    var node_dict = {};
    // Find parents for all nodes once
    (function(){
      for (var i in trimmed_nodes) {
        var node = trimmed_nodes[i];
        node_dict[node.name] = node;
        for (var j in node.child_names) {
          parents[node.child_names[j]] = node.name;
        }
      }
    })();

    var root = d3
        .stratify()
        .id(function(node) {
          return node.name;
        })
        .parentId(function(node) {
          // undefined if it has no parent - does that break the layout?
          if (node.name in parents) {
            return parents[node.name];
          } else if (node.name === forest_root.name) {
            return undefined;
          } else {
            forest_root.child_names.push(node.name);
            return forest_root.name;
          }
        })(trimmed_nodes);

    root.sort(function(a, b) {
      if (a.depth !== b.depth) {
        return b.depth - a.depth;
      }
      while (a.parent !== b.parent) {
        a = a.parent;
        b = b.parent;
      }
      var child_list = a.parent.data.child_names;
      return (
        child_list.findIndex(x => x === a.data.name) -
          child_list.findIndex(x => x === b.data.name)
      );
    });


    var svg = d3.select(this.svg_ref.current);


    var container = d3.select(this.viewport_ref.current);
    var width = container.attr("width"),
        height = container.attr("height");


    var g_edge = svg.select("g.edges");
    var g_vertex = svg.selectAll("g.vertices");
    var g_data = svg.selectAll("g.data_graph");
    var g_droptargets = svg.selectAll("g.drop_targets");

    var node = g_vertex
        .selectAll(".node")
        .data(root.descendants()
              .filter(node => node.id !== forest_root.name),
              function(node) {
                return node.id;
              });

    node.exit().remove();

    node = node
      .enter()
      .call(this.drawNodes.bind(this))
      .merge(node)
      .selectAll(".btnode")
      .data(x => [x], x => x.id)
      .call(this.updateNodes.bind(this));

    // TODO(nberg): Find a way to get rid of this - it's here because
    // the DOM changes in updateNodes take a while to actually happen,
    // and layoutNodes needs getBoundingClientRect information...
    window.setTimeout(
      () =>
        {
          this.layoutNodes(svg, width, height, root);

          this.drawDropTargets();

          this.drawDataGraph(g_data, node.data(), tree_msg.data_wirings);
        }, 100);

    //console.log(root);
  }

  drawNodes(selection)
  {
    selection.each(function(d) {
      d._entering = true;
      d._show = true;
      d.x = 0;
      d.y = 0;
    });

    var that = this;
    var fo = selection.append('foreignObject')
        .attr("class", function(d) {
          return "node" + (d.children ? " node--internal" : " node--leaf");
        })
        .on("click", this.nodeClickHandler.bind(this))
        .on("mousedown", function(d, index, group) {
          that.nodeMousedownHandler(d, this);
        })
        .on("dblclick", this.nodeDoubleClickHandler.bind(this));

    var div = fo
        .append("xhtml:body")
        .attr("class", "btnode p-2")
        .style("min-height", d => {
          // We need to ensure a minimum height, in case the node body
          // would otherwise be shorter than the number of grippers
          // requires.
          var inputs = d.data.inputs || [];
          var outputs = d.data.outputs || [];
          var max_num_grippers = Math.max(inputs.length, outputs.length);
          return ((this.io_gripper_size + this.io_gripper_spacing) * max_num_grippers) + 'px';
        });
  }

  updateNodes(selection)
  {
    // Update name
    var title = selection.selectAll(".node_name").data(function(d) {
      return [d];
    });
    title = title.enter().append("h4").attr("class", "node_name").merge(title);
    title.html(function(d) {
      return d.id;
    });

    var className = selection.selectAll(".class_name").data(function(d) {
      return [d];
    });
    className = className.enter().append("h5").attr("class", "class_name").merge(className);
    className.html(function(d) {
      return d.data.node_class;
    });
  }

  fillTables (tbody)
  {
    var rows = tbody.selectAll(".node_data")
        .data(function(d) {
          return d.value;
        }, d => d.key);

    rows.exit().remove();
    // create new rows
    rows = rows.enter().append("tr").attr("class", "node_data").merge(rows);

    var keys = rows.selectAll(".key").data(d => [d.key]);
    keys.exit().remove();
    keys = keys.enter().append("td").attr("class", "key").merge(keys);
    keys.text(d => d);

    var values = rows.selectAll(".value").data(d => [d.serialized_value]);
    values.exit().remove();
    values = values.enter().append("td").attr("class", "value").merge(values);
    values.text(d => d);
  }

  layoutNodes(svg, width, height, root)
  {
    var g_edge = svg.select("g.edges");
    var g_vertex = svg.selectAll("g.vertices");

    var nodes_without_forest_root = root.descendants()
        .filter(node => node.id !== '__forest_root');
    // k is the zoom level - we need to apply this to the values we get
    // from getBoundingClientRect, or we get fun scaling effects.
    var zoom = d3.zoomTransform(d3.select(this.viewport_ref.current).node()).k;

    // Find the maximum size of all the nodes, for layout purposes
    var max_size = [0,0];
    var max_height_by_depth = Array(root.height + 1).fill(0.0);
    g_vertex.selectAll('.btnode')
      .data(nodes_without_forest_root,
            x => x.id)
      .each(function(d, index){
        var rect = this.getBoundingClientRect();
        rect.x /= zoom;
        rect.y /= zoom;
        rect.width /= zoom;
        rect.height /= zoom;
        d._size = rect;
        max_height_by_depth[d.depth] = Math.max(max_height_by_depth[d.depth], rect.height);
        this.parentElement.setAttribute('width', rect.width);
        this.parentElement.setAttribute('height', rect.height);
      });

    var tree_size = [width - max_size[0], height - (40 + max_size[1])];

    var tree = d3.flextree()
        .nodeSize(function(node) {
          if (node._size)
          {
            return [node._size.width + this.spacing,
                    max_height_by_depth[node.depth] + this.spacing];
          }
          else
          {
            return [1,1];
          }
        }.bind(this))
    (root);

    // Move new nodes to their starting positions
    g_vertex.selectAll(".node")
      .filter(d => d._entering)
      .attr("transform", function(d) {
        // Start at parent position
        var p = this.findExistingParent(d);
        return "translate(" + Math.round(p.x) + "," + Math.round(p.y) + ") scale(0.1)";
      }.bind(this));

    var link = g_edge.selectAll(".link")
        .data(tree.links()
              .filter(x=>x.source.id !== '__forest_root' && x.target.id !== '__forest_root'),
              function(d) { return '' + d.source.id + d.target.id; });
    link.exit().remove();

    link = link
      .enter().append("path")
      .attr("class", "link")
      .attr("d", d3.linkVertical()
            .source(function(d) {
              var parent = this.findExistingParent(d.source);
              return [Math.round(parent.x), Math.round(parent.y + parent._size.height)];
            }.bind(this))
            .target(function(d) {
              var parent = this.findExistingParent(d.target);
              return [Math.round(parent.x), Math.round(parent.y)];
            }.bind(this)))
      .merge(link);

    g_vertex.selectAll(".node").each(function(d) {
      d._entering = false;
    });

    link.transition()
      .duration(250).
      attr("d", d3.linkVertical()
           .source(function(d) {
             return [Math.round(d.source.x), Math.round(d.source.y + d.source._size.height)];
           })
           .target(function(d) {
             return [Math.round(d.target.x), Math.round(d.target.y)];
           }));


    // new selection, now with the elements we just added with enter()
    // above
    var node = g_vertex.selectAll(".node")
      .data(root.descendants(), function(node) {return node.id;});

    var t = d3.transition()
        .duration(250);
    node.transition(t)
      .attr("transform", function(d) {
        // animate to actual position
        return "translate(" + Math.round(d.x - d._size.width / 2.0) + "," + Math.round(d.y) + ") scale(1.0)";
      });
        node
      .selectAll(".btnode")
      .transition(t)
      .ease(d3.easeQuad)
    // Update color based on node state
      .style("border-color", function(d) {
        switch (d.data.state) {
        case "RUNNING": {
          return "#ffc107";
        }
        case "IDLE":{
          return "#007bff";
        }
        case "SUCCEEDED": {
          return "#28a745";
        }
        case "FAILED": {
          return "#dc3545";
        }
        case "DEBUG_PRE_TICK":
        case "DEBUG_POST_TICK":
        case "DEBUG_TICK": {
          return "#17a2b8";
        }
        case "SHUTDOWN": {
          return "#7c1e27";
        }
        case "UNINITIALIZED":
        default: {
          return "#4E5666";
        }
        };
      });
  }

  findExistingParent(d)
  {
    while (d._entering && d.parent && d.parent._size) {
      d = d.parent;
    }
    return d;
  }

  // DATA GRAPH HELPERS
  getGripperSize(height, inputs, outputs)
  {
    return Math.min(
      this.max_io_gripper_size,
      // +1 to ensure nice padding on the bottom
      outputs != 0 ?
        ((height - ((outputs + 1) * this.io_gripper_spacing)) / outputs)
        :
        this.max_io_gripper_size,
      inputs != 0 ?
        ((height - ((inputs + 1) * this.io_gripper_spacing)) / inputs)
        :
        this.max_io_gripper_size);
  }

  getGripperCoords(index, right, gripper_size, node_width)
  {
    return {
      x: 0.5 * node_width - (right ? 0.0 : gripper_size + node_width),
      y: this.io_gripper_spacing + (index * (this.io_gripper_spacing + gripper_size))
    };
  }

  drawDropTargets()
  {
    var g_droptargets = d3.select(this.svg_ref.current).select("g.drop_targets");

    // We can't really assign keys to the targets, so remove them all :/
    g_droptargets.selectAll(".drop_target").remove();

    // For each node, decide what kinds of drop targets to add.
    //
    // Possible drop targets are:
    //
    // Neighbors (left/right):
    // Insert the dropped node into the parent's list of children
    // before or after this node.
    // Only available if the parent doesn't have its maximum number of children yet!
    //
    // Replace (bounding rect of the node itself):
    // Replace this node with the dropped node.
    //
    // Below:
    // Add the dropped node as this node's child - only available if there's 0 children!
    //
    // Above:
    // Add the dropped node as this node's parent, taking over its old position.
    // Note that this is realized by two service calls:
    // 1. Move this node to be dropped node's child
    // 2. Move dropped node to this node's position
    d3.select(this.svg_ref.current).select("g.vertices").selectAll(".node").each(
      function(d)
      {
        // No drop options at the root of the tree (__forest_root)!
        if (!d.parent ) {
          return;
        }

        var my_index = d.parent.children.findIndex(x => x.data.name == d.data.name);

        // Only add the "insert before" target for the first node.
        // Without this if, we'd get overlapping drop targets for
        // this.after and next_child.before
        if (my_index == 0)
        {
          g_droptargets
            .append("rect")
            .attr("class", "drop_target")
            .attr("transform",
                  "translate(" + (d.x - this.spacing - (d._size.width * 0.5)) + ","
                  + d.y + ")")
            .attr("width", this.spacing)
            .attr("height", d._size.height)
            .datum({
              position: my_index,  // insert before this node
              replace: false,
              data: d.parent.data
            });
        }
        // Right drop target
        g_droptargets
          .append("rect")
          .attr("class", "drop_target")
          .attr("transform",
                "translate(" + (d.x + (d._size.width * 0.5)) + ","
                + d.y + ")")
          .attr("width", this.spacing)
          .attr("height", d._size.height)
          .datum({
            position: my_index + 1,  // insert after this node
            replace: false,
            data: d.parent.data
          });

        // Center drop target (on a node)
        g_droptargets
          .append("rect")
          .attr("class", "drop_target")
          .attr("transform",
                "translate(" + (d.x - (d._size.width * 0.5)) + ","
                + d.y + ")")
          .attr("width", d._size.width)
          .attr("height", d._size.height)
          .datum({
            position: -1,
            replace: true,  // replace this node
            data: d.data
          });

        // Top drop target
        g_droptargets
          .append("rect")
          .attr("class", "drop_target")
          .attr("transform",
                "translate(" + (d.x - (d._size.width * 0.5)) + ","
                + (d.y - (this.spacing * 0.5)) + ")")
          .attr("width", d._size.width)
          .attr("height", this.spacing * 0.45)
          .datum({
            // replace the node at the given index from data, and take
            // it as our own child
            position: my_index,
            replace: true,
            data: d.parent.data
          });

        //Bottom drop target
        var child_names = d.data.child_names || [];
        var max_children = d.data.max_children;
        if (max_children === undefined) {
          max_children = -1;
        }

        // If max_children is either -1 or >0, we can add a child
        if (max_children != 0 || child_names.length < max_children)
        {
          g_droptargets
            .append("rect")
            .attr("class", "drop_target")
            .attr("transform",
                  "translate(" + (d.x - (d._size.width * 0.5)) + ","
                  + (d.y + d._size.height) + ")")
            .attr("width", d._size.width)
            .attr("height", this.spacing * 0.45)
            .datum({
              // Add as first child of this node
              position: 0,
              replace: false,
              data: d.data
            });
        }
      }.bind(this));

    var that = this;
    g_droptargets.selectAll(".drop_target")
      .attr("opacity", 0.2)
      .on("mouseover", function(d, index, group)
          {
            that.dropTargetDefaultMouseoverHandler(this, d);
          })
      .on("mouseout", function(d, index, group)
          {
            that.dropTargetDefaultMouseoutHandler(this, d);
          });
  }

  drawDataGraph(g_data, data, wirings)
  {
    var edges = g_data.select("g.data_edges");
    var vertices = g_data.select("g.data_vertices");

    var input_vertex_data = [];
    var output_vertex_data = [];
    data.forEach(function(x) {
      Array.prototype.push.apply(
        input_vertex_data,
        x.data.inputs.map(
          function(input) {
            var datum = this.getIOCoordsFromNode(
              x,
              input.key,
              "inputs",
              /*centered=*/false);
            datum.nodeName = x.data.name;
            datum.key = input.key;
            datum.kind = "input";
            datum.type = input.serialized_type;
            return datum;
          }.bind(this)));

      Array.prototype.push.apply(
        output_vertex_data,
        x.data.outputs.map(
          function(output) {
            var datum = this.getIOCoordsFromNode(
              x,
              output.key,
              "outputs",
              /*centered=*/false);
            datum.nodeName = x.data.name;
            datum.key = output.key;
            datum.kind = "output";
            datum.type = output.serialized_type;
            return datum;
          }.bind(this)));
    }.bind(this));

    this.drawDataVerts(vertices, input_vertex_data, output_vertex_data);

    this.drawDataEdges(
      edges,
      wirings.map(function(wiring) {
        var start = this.getIOCoords(data,
                                     wiring.source.node_name,
                                     wiring.source.data_kind,
                                     wiring.source.data_key,
                                     /*centered=*/true);

        var two = {
          x : start.x ,
          y : start.y - 2
        };

        if (wiring.source.data_kind === "inputs")
        {
          two.x = two.x - 10;
        }
        else if (wiring.source.data_kind === "outputs")
        {
          two.x = two.x + 10;
        }
        else
        {
          // two.y = two.y - 10;
        }

        var target = this.getIOCoords(data,
                                      wiring.target.node_name,
                                      wiring.target.data_kind,
                                      wiring.target.data_key,
                                      /*centered=*/true);

        var three = {
          x : target.x ,
          y : target.y - 2
        };

        if (wiring.target.data_kind === "inputs")
        {
          three.x = three.x - 10;
        }
        else if (wiring.target.data_kind === "outputs")
        {
          three.x = three.x + 10;
        }
        else
        {
          // three.y = three.y - 10;
        }

        return {
          source: {
            node_name: wiring.source.node_name,
            data_kind: wiring.source.data_kind,
            data_key: wiring.source.data_key
          },
          target: {
            node_name: wiring.target.node_name,
            data_kind: wiring.target.data_kind,
            data_key: wiring.target.data_key
          },
          points: [
            start,
            two,
            three,
            target
          ]};
      }.bind(this)));
  }

  getIOCoords(node_data,
              node_name,
              data_kind,
              data_key,
              centered)
  {
    var node = node_data.find(d => d.data.name === node_name);
    return this.getIOCoordsFromNode(node, data_key, data_kind, centered);
  }

  getIOCoordsFromNode(
    node,
    data_key,
    data_kind,
    centered)
  {
    centered = centered || false;

    if (!node)
    {
      // Shouldn't really happen...
      return {x: 0, y: 0, gripperSize: 0};
    }

    var coords = null;
    var inputs = node.inputs || [];
    var outputs = node.outputs || [];

    if (data_kind === 'inputs')
    {
      coords = this.getGripperCoords(
        node.data.inputs.findIndex(x => x.key === data_key) || 0,
        /*right=*/false,
        this.io_gripper_size,
        node._size.width);
    }
    else if (data_kind === 'outputs')
    {
      coords = this.getGripperCoords(
        node.data.outputs.findIndex(x => x.key === data_key) || 0,
        /*right=*/true,
        this.io_gripper_size,
        node._size.width);
    }
    else
    {
      // For things that are neither inputs nor outputs, just draw a
      // line to the center of the node
      coords = {
        x: 0.5 * node._size.width,
        y: 0.5 * node._size.height
      };
    }

    if (centered)
    {
      coords.x += this.io_gripper_size * 0.5;
      coords.y += this.io_gripper_size * 0.5;
    }
    return {
      x: node.x + coords.x,
      y: node.y + coords.y,
      gripperSize: this.io_gripper_size
    };
  }

  drawDataEdges(edge_selection, edge_data)
  {
    var link = edge_selection.selectAll(".data-link").data(
      edge_data,
      d => JSON.stringify(d.source) + JSON.stringify(d.target));
    link.exit().remove();

    link = link
      .enter()
      .append("path")
      .attr("class", "data-link")
      .on("click", this.DataEdgeDefaultClickHandler.bind(this))
      .on("mouseover", this.DataEdgeDefaultMouseoverHandler)
      .on("mouseout", this.DataEdgeDefaultMouseoutHandler)
      .merge(link);

    link
      .transition()
      .attr("d", function(d) {
        var lineGen = d3.line()
            .x(d => d.x)
            .y(d => d.y)
            .curve(d3.curveCatmullRom.alpha(0.9));
        return lineGen(d.points);
      });
  }

  drawDataVerts(vertex_selection, input_vertex_data, output_vertex_data)
  {
    var groups = vertex_selection.selectAll(".gripper-group").data(
      input_vertex_data.concat(output_vertex_data),
      d => d.nodeName + d.kind + d.key);
    groups.exit().remove();

    groups = groups
      .enter()
      .append("g")
      .attr("class", d => "gripper-group " + d.kind + "-gripper-group")
      .on("mouseover.highlight", this.IOGroupDefaultMouseoverHandler)
      .on("mouseout.highlight", this.IOGroupDefaultMouseoutHandler)
      .merge(groups);

    groups
      .transition()
      .attr("transform", function(d) {
        return "translate(" + Math.round(d.x) + ", " + Math.round(d.y) + ")";
      });

    var grippers = groups.selectAll(".gripper").data(d=> [d]);
    grippers.exit().remove();

    grippers = grippers
      .enter()
      .append("rect")
      .attr("class", d => "gripper " + d.kind + "-gripper")
      .attr("width", d => d.gripperSize)
      .attr("height", d => d.gripperSize)
      .on("mousedown", this.IOGripperMousedownHandler.bind(this))
      .merge(grippers);

    var labels = groups.selectAll(".label").data(d=> [d]);
    labels.exit().remove();

    labels = labels
      .enter()
      .append("text")
      .attr("class", "label")
      .attr("text-anchor", d => d.kind === "input" ? "end" : "start")
      .attr("dominant-baseline", "middle")
      .attr("visibility", "hidden")
      .attr("dx", d => {
        if (d.kind === "input") {
          return Math.round(-5);
        }
        else if (d.kind === "output") {
          return Math.round(d.gripperSize + 5);
        }
        return 0;
      })
      .attr("dy", d => Math.round(0.5 * d.gripperSize))
      .merge(labels);
    labels.text(d => d.key + " (type: " + prettyprint_type(d.type) + ")");
  }

  IOGroupDefaultMouseoverHandler(d, index, group)
  {
    d3.select(this).classed("data-hover", true)
      .selectAll(".label")
      .attr("visibility", "visible");
  }

  IOGroupDefaultMouseoutHandler(d, index, group)
  {
    d3.select(this).classed("data-hover", false)
      .selectAll(".label")
      .attr("visibility", "hidden");
  }

  IOGroupDraggingMouseoverHandler(d, index, group)
  {
    if (d.kind === "input")
    {
      this.nextWiringTarget = d;
    }
    else if (d.kind === "output") {
      this.nextWiringSource = d;
    }
  }

  IOGroupDraggingMouseoutHandler(d, index, group)
  {
    if (d.kind === "input")
    {
      this.nextWiringTarget = null;
    }
    else if (d.kind === "output") {
      this.nextWiringSource = null;
    }
  }

  DataEdgeDefaultMouseoverHandler(d, index, group)
  {
    var vertex_selection = d3.select(this.parentNode.parentNode)
        .select("g.data_vertices");
    d3.select(this).classed("data-hover", true);

    // select source gripper
    selectIOGripper(vertex_selection, d.source)
      .dispatch("mouseover");

    // select target gripper
    selectIOGripper(vertex_selection, d.target)
      .dispatch("mouseover");
  }

  DataEdgeDefaultMouseoutHandler(d, index, group)
  {
    var vertex_selection = d3.select(this.parentNode.parentNode)
        .select("g.data_vertices");

    d3.select(this).classed("data-hover", false);

    // deselect source gripper
    selectIOGripper(vertex_selection, d.source)
      .dispatch("mouseout");

    // deselect target gripper
    selectIOGripper(vertex_selection, d.target)
      .dispatch("mouseout");
  }

  DataEdgeDefaultClickHandler(d, index, group)
  {
    this.props.onSelectedEdgeChange(d);
    d3.event.preventDefault();
    d3.event.stopPropagation();
  }

  IOGripperMousedownHandler(datum, index, group)
  {
    if ((d3.event.buttons & 1) != 1)
    {
      return;
    }
    // Remove mouseover / out listeners from all gripper-groups, then add new ones
    var svg_sel = d3.select(this.svg_ref.current);
    var io_grippers = svg_sel.selectAll(".gripper-group");

    io_grippers
      .on("mouseover", null)
      .on("mouseout", null);

    // Remove mouseover listener from data edges so we don't
    // accidentally send a mouseover event to a gripper while crossing
    // an edge connected to it
    svg_sel.selectAll('.data-link')
      .on("mouseover", null)
      .on("mouseout", null);

    // Hide this gripper's label
    svg_sel.selectAll(".label").datum(datum).attr("visibility", "hidden");

    // Save the datum to use in the wire request later
    if (datum.kind === "input")
    {
      this.nextWiringTarget = datum;
    }
    else if (datum.kind === "output") {
      this.nextWiringSource = datum;
    }

    // Also save the current mouse position (relative to the viewport
    // <g> tag)
    this.dragStartPos = d3.mouse(this.svg_ref.current);

    // Give compatible IOs a new listener and highlight them
    io_grippers
      .filter(d => typesCompatible(d, datum))
      .classed("compatible", true)
      .on("mouseover.drag",
          this.IOGroupDraggingMouseoverHandler.bind(this))
      .on("mouseout.drag",
          this.IOGroupDraggingMouseoutHandler.bind(this))
      .selectAll(".label")
      .attr("visibility", "visible");

    this.dragging = true;
    // Give the canvas a move and mouseup handler
    d3.select(this.viewport_ref.current)
    // Remove any handlers for node dragging
      .on("mousemove.drag_node", null)
      .on("mouseup.drag_node", null)
      .on("mousemove.drag_io", this.canvasIOMoveHandler.bind(this))
      .on("mouseup.drag_io", this.canvasIOUpHandler.bind(this));

    d3.event.preventDefault();
    d3.event.stopPropagation();
  }

  canvasMousemovePanHandler(d, index, group)
  {
    // Don't do anything unless we're currently dragging something
    if (!this.dragging){
      if (this.panIntervalID !== null)
      {
        window.clearInterval(this.panIntervalID);
        this.panDirection = [];
      }

      return;
    }

    var viewport = d3.select(this.viewport_ref.current);
    var width = viewport.node().getBoundingClientRect().width;
    var height = viewport.node().getBoundingClientRect().height;

    var mouseCoords = d3.mouse(this.viewport_ref.current);

    var panNeeded = false;
    this.panDirection = [0.0, 0.0];

    if (mouseCoords[0] < this.dragPanBoundary) {
      // Left edge -> scroll right
      panNeeded = true;
      // 1 if mouse is at the left edge, 0 if it is dragPanBoundary pixels away
      var leftEdgeCloseness = (this.dragPanBoundary - mouseCoords[0]) / this.dragPanBoundary;
      this.panDirection[0] = this.panPerFrame * leftEdgeCloseness;
    } else if (mouseCoords[0] > (width - this.dragPanBoundary)) {
      // Right edge -> scroll left
      panNeeded = true;
      // 1 if mouse is at the right edge, 0 if it is dragPanBoundary pixels away
      var rightEdgeCloseness = (this.dragPanBoundary - (width - mouseCoords[0])) / this.dragPanBoundary;
      this.panDirection[0] = -1.0 * this.panPerFrame * rightEdgeCloseness;
    }
    if (mouseCoords[1] < this.dragPanBoundary) {
      // Up -> scroll down
      panNeeded = true;
      // 1 if mouse is at the top edge, 0 if it is dragPanBoundary pixels away
      var topEdgeCloseness = (this.dragPanBoundary - mouseCoords[1]) / this.dragPanBoundary;
      this.panDirection[1] = this.panPerFrame * topEdgeCloseness;
    } else if (mouseCoords[1] > (height - this.dragPanBoundary)) {
      // Down -> scroll up
      panNeeded = true;
      // 1 if mouse is at the bottom edge, 0 if it is dragPanBoundary pixels away
      var botEdgeCloseness = (this.dragPanBoundary - (height - mouseCoords[1])) / this.dragPanBoundary;
      this.panDirection[1] = -1.0 * this.panPerFrame * botEdgeCloseness;
    }

    if (!panNeeded && this.panIntervalID !== null)
    {
      window.clearInterval(this.panIntervalID);
      this.panIntervalID = null;
      return;
    }

    if (this.panIntervalID === null)
    {
      // Start the interval for the panning animation, at panRate Hz
      this.panIntervalID = window.setInterval(this.dragPanTimerHandler, 1000.0 / this.panRate);
    }
  }

  dragPanTimerHandler()
  {
    if (!this.dragging && this.panIntervalID)
    {
      window.clearInterval(this.panIntervalID);
      this.panIntervalID = null;
      return;
    }

    d3.select(this.viewport_ref.current)
      .call(this.zoomObject.translateBy, this.panDirection[0], this.panDirection[1]);
  }

  canvasIOMoveHandler(d, index, group)
  {
    if ((d3.event.buttons & 1) === 0)
    {
      this.canvasIOUpHandler(d, index, group);
      return;
    }

    var drawingLine = d3
        .select(this.svg_ref.current)
        .selectAll(".drawing-indicator")
        .data(
          [{
            start: this.dragStartPos,
            end: d3.mouse(this.svg_ref.current)
          }],
          /*key=*/d => JSON.stringify(d.start));

    drawingLine.exit().remove();
    drawingLine = drawingLine
      .enter()
      .append("path")
      .attr("class", "drawing-indicator")
      .merge(drawingLine);

    drawingLine
      .attr("d", data => d3.line()([data.start, data.end]));
    d3.event.preventDefault();
  }

  canvasIOUpHandler(d, index, group)
  {
    this.dragging = false;

    if (this.nextWiringSource && this.nextWiringTarget)
    {
      this.wire_service.callService(
        new ROSLIB.ServiceRequest({
          wirings: [
            {
              source: {
                node_name: this.nextWiringSource.nodeName,
                data_kind: this.nextWiringSource.kind + "s", // should be inputs!
                data_key: this.nextWiringSource.key
              },
              target: {
                node_name: this.nextWiringTarget.nodeName,
                data_kind: this.nextWiringTarget.kind + "s", // should be outputs!
                data_key: this.nextWiringTarget.key
              }
            }]
        }),
        function(response) {
          if (response.success)
          {
            console.log("Successfully wired data!");
          }
          else
          {
            this.props.onError("Failed to wire data " + this.nextWiringSource +
                               " to " + this.nextWiringTarget + ": " + JSON.stringify(response));
          }
        }.bind(this));
    }

    // Either way, we're done dragging, so restore the old handlers

    // Remove mouseover / out listeners from all gripper-groups, then
    // add back the default ones
    var io_grippers = d3.select(this.svg_ref.current).selectAll(".gripper-group");
    io_grippers
      .on("mouseover", null)
      .on("mouseout", null)
    // Also remove this class again
      .classed("data-hover", false)
      .classed("compatible", false);

    io_grippers
    // Remove the drag listeners
      .on("mouseover.drag", null)
      .on("mouseout.drag", null)
    // And hide the labels again
      .selectAll(".label")
      .attr("visibility", "hidden");


    d3.select(this.svg_ref.current).selectAll('.data-link')
      .on("mouseover", this.DataEdgeDefaultMouseoverHandler)
      .on("mouseout", this.DataEdgeDefaultMouseoutHandler);

    // Also remove listeners from the background
    d3.select(this.viewport_ref.current)
      .on("mousemove.drag_io", null)
      .on("mouseup.drag_io", null);

    // Remove the drawing line from the DOM
    d3.select(this.viewport_ref.current).selectAll(".drawing-indicator").data([])
      .exit().remove();

    // Finally, remove these:
    this.nextWiringSource = null;
    this.nextWiringTarget = null;
  }

  canvasNodeDragMoveHandler()
  {
    if (this.draggedNode === null)
    {
      return;
    }
    var newly_dragging = (
      !this.dragging
        && getDist(d3.mouse(this.draggedNode.domObject),
                   this.draggedNode.startCoords) > this.min_node_drag_distance);

    if (newly_dragging)
    {
      this.dragging = true;
      // Hide all drop targets that would lead to appending the dragged
      // node to its own subtree
      var d = this.draggedNode.data;
      var parentName = d.parent ? d.parent.data.name || '' : '';
      var my_index = d.parent.children.findIndex(x => x.data.name == d.data.name);

      var svg = d3.select(this.svg_ref.current);

      var all_children = d.descendants().map(x => x.data.name);

      var g_droptargets = svg.select("g.drop_targets");

      g_droptargets.attr("visibility", "visible");

      g_droptargets.selectAll(".drop_target")
      // First ensure all drop targets are visible
        .attr("visibility", "visible")
      // Now hide those that belong to descendants of the node we're dragging
        .filter(
          x => {
            // Hide drop targets of children
            if (all_children.indexOf(x.data.name) >= 0) {
              return true;
            }
            // Hide the left/right drop targets
            if (x.data.name === parentName
                && (x.position == my_index
                    || x.position == my_index + 1)) {
              return true;
            }
            // Hide the left/right drop targets for nodes that are
            // children of a *different* parent than ours, with
            // max_children children. (if its our own parent, moving
            // us won't change the number of children, so we're okay)
            var child_names = x.data.child_names || [];
            var max_children = x.data.max_children || -1;
            if (x.data.name !== parentName &&
                max_children !== -1 &&
                child_names.length === max_children) {
              return true;
            }
            return false;
          })
        .attr("visibility", "hidden");
    }

    if ((d3.event.buttons & 1) === 0)
    {
      this.canvasNodeDragUpHandler(d, index, group);
    }

    d3.event.preventDefault();
    d3.event.stopPropagation();
  }

  canvasNodeDragUpHandler(d, index, group)
  {
    if (this.dragging)
    {
      if (this.nodeDropTarget)
      {
        // Calculate the final index to move the dropped node to.
        //
        // If the target is in the same parent node, and after the
        // current index of the dropped node, subtract one from the
        // target index to make the behavior more intuitive

        if (this.nodeDropTarget.position > 0
            && this.nodeDropTarget.data.name === this.draggedNode.data.parent.data.name
            && this.draggedNode.data.parent.data.child_names.indexOf(
              this.draggedNode.data.data.name) < this.nodeDropTarget.position)
        {
          this.nodeDropTarget.position -= 1;
        }

        // Also replace __forest_root with the empty string if present
        if (this.nodeDropTarget.data.name === "__forest_root")
        {
          this.nodeDropTarget.data.name = "";
        }

        // Three possible cases:

        // 1. Valid position, replace == false
        //    In this case we just call MoveNode with the given
        //    parent node and index
        if (this.nodeDropTarget.position >= 0
            && !this.nodeDropTarget.replace)
        {
          this.move_service.callService(
            new ROSLIB.ServiceRequest({
              node_name: this.draggedNode.data.data.name,
              new_parent_name: this.nodeDropTarget.data.name,
              new_child_index: this.nodeDropTarget.position
            }),
            function(response)
            {
              if (response.success)
              {
                console.log("Successfully moved node!");
              }
              else
              {
                this.props.onError("Failed to move node: " + response.error_message);
              }
            }.bind(this));
        }
        // 2. Valid position, replace == true
        //    First, remove the dropped node from its parent to prevent cycles.
        //    Then, move the node at the selected position to be our child,
        //    and finally move the dropped node to the indicated position as in 1.
        else if (this.nodeDropTarget.position >= 0
                 && this.nodeDropTarget.replace)
        {
          this.move_service.callService(
            new ROSLIB.ServiceRequest({
              node_name: this.draggedNode.data.data.name,
              new_parent_name: ''
            }),
            function(response)
            {
              if (response.success)
              {
                console.log("Successfully removed dropped node from its parent!");
                this.move_service.callService(
                  new ROSLIB.ServiceRequest({
                    // nodeDropTarget is the *parent* of the node we want to move!
                    node_name: this.nodeDropTarget.data.child_names[this.nodeDropTarget.position],
                    new_parent_name: this.draggedNode.data.data.name,
                    new_child_index: -1
                  }),
                  function(response)
                  {
                    if (response.success)
                    {
                      console.log("Successfully moved old node to be a child of dropped node, "
                                  + "now moving dropped node!");
                      this.move_service.callService(
                        new ROSLIB.ServiceRequest({
                          node_name: this.draggedNode.data.data.name,
                          // Now the parent of the node we moved first will become our parent!
                          new_parent_name: this.nodeDropTarget.data.name,
                          new_child_index: this.nodeDropTarget.position
                        }),
                        function(response)
                        {
                          if (response.success)
                          {
                            console.log("Successfully moved dropped node to the place "
                                        + "the old node was in!");
                          }
                          else
                          {
                            this.props.onError("Failed to move node: " + response.error_message);
                          }
                        }.bind(this));
                    }
                    else
                    {
                      this.props.onError("Failed to move node: " + response.error_message);
                    }
                  }.bind(this));
              }
              else
              {
                this.props.onError("Failed to move node: " + response.error_message);
              }
            }.bind(this));
        }
        // 3. position === -1, replace == true
        //    Call ReplaceNode with the given node name
        else if (this.nodeDropTarget.position == -1
                 && this.nodeDropTarget.replace)
        {
          this.replace_service.callService(
            new ROSLIB.ServiceRequest({
              old_node_name: this.nodeDropTarget.data.name,
              new_node_name: this.draggedNode.data.data.name
            }),
            function(response)
            {
              if (response.success)
              {
                console.log("Successfully replaced old node with dropped node!");
              }
              else
              {
                this.props.onError("Failed to replace node: " + response.error_message);
              }
            }.bind(this));
        }
        else
        {
          this.props.onError("Unexpected data for dropping a node :(");
        }
        d3.event.preventDefault();
        d3.event.stopPropagation();
      }
      else
      {
        console.log("should trigger the click event, hopefully?");
      }
    }

    this.dragging = false;

    d3.select(this.svg_ref.current).select("g.drop_targets")
      .attr("visibility", "hidden")
      .selectAll(".drop_target")
      .attr("visibility", "");

    d3.select(this.viewport_ref.current)
      .on("mousemove.drag_node", null)
      .on("mouseup.drag_node", null);
  }

  nodeClickHandler(d, index, group)
  {
    this.props.onSelectionChange(d.data.name);
    d3.event.preventDefault();
    d3.event.stopPropagation();
  }

  nodeDoubleClickHandler(d, index, group)
  {
    if (d.data.module === "ros_bt_py.nodes.subtree" && d.data.node_class === "Subtree")
    {
      var selected_subtree = this.props.subtreeNames.filter((subtree) => subtree.startsWith(d.data.name+"."));
      if (selected_subtree.length == 1)
      {
        this.props.onSelectedTreeChange(
          /*is_subtree=*/ true,
          /*name=*/ selected_subtree[0]);
      } else {
        if (this.props.publishing_subtrees)
        {
          this.props.onError("Selected subtree does not exist, this should not happen.");
        } else {
          this.props.onError("Cannot show a Subtree that is not published. Please enable \"Publish Subtrees\"!")
        }
      }
    }
    d3.event.preventDefault();
    d3.event.stopPropagation();
  }

  nodeMousedownHandler(d, domObject)
  {
    if ((d3.event.buttons & 1) != 1)
    {
      return;
    }

    // No dragging the fake root!
    if (d.data.name == "__forest_root")
    {
      return;
    }

    this.draggedNode = {
      startCoords: d3.mouse(domObject),
      domObject: domObject,
      data: d
    };

    this.dragging = false;

    // Add move and mouseup handlers to viewport, and just to be sure,
    // remove the drag_io handlers in case they're present
    d3.select(this.viewport_ref.current)
      .on("mousemove.drag_io", null)
      .on("mouseup.drag_io", null)
      .on("mousemove.drag_node", this.canvasNodeDragMoveHandler.bind(this))
      .on("mouseup.drag_node", this.canvasNodeDragUpHandler.bind(this));

    d3.event.preventDefault();
    d3.event.stopPropagation();
  }

  dropTargetDefaultMouseoverHandler(domElement, datum)
  {
    this.nodeDropTarget = datum;
    d3.select(domElement).attr("opacity", 0.8);
  }

  dropTargetDefaultMouseoutHandler(domElement, datum)
  {
    this.nodeDropTarget = null;
    d3.select(domElement).attr("opacity", 0.2);
  }

  resetView()
  {
    var viewport = d3.select(this.viewport_ref.current);
    var height = viewport.node().getBoundingClientRect().height;

    var container = d3.select(this.svg_ref.current);

    viewport
      .call(this.zoomObject.scaleExtent([0.3, 1.0]).on("zoom", function () {
        container.attr("transform", d3.event.transform);
      }))
      .call(this.zoomObject.translateTo, 0.0, (height * 0.5) - 10.0);
  }
}

class NewNode extends Component
{
  constructor(props)
  {
    super(props);

    if (props.node) {
      this.state = {
        name: props.node.name,
        isValid: true,
        options: this.getDefaultValues(props.node.options),
        inputs: this.getDefaultValues(props.node.inputs,
                                      props.node.options),
        outputs: this.getDefaultValues(props.node.outputs,
                                       props.node.outputs)
      };
    }
    else
    {
      this.state = {
        name: '',
        isValid: false,
        options: [],
        inputs: [],
        outputs: []
      };
    }

    this.add_node_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'add_node',
      serviceType: 'ros_bt_py_msgs/AddNode'
    });

    this.selectRef = React.createRef();

    this.nameChangeHandler = this.nameChangeHandler.bind(this);
    this.updateValidity = this.updateValidity.bind(this);
    this.updateValue = this.updateValue.bind(this);
    this.onClickAdd = this.onClickAdd.bind(this);
  }

  render()
  {
    return(
      <div className="d-flex flex-column">
        <button className="btn btn-block btn-primary"
                disabled={!this.state.isValid}
                onClick={this.onClickAdd}>Add to Tree</button>
        <label className="pt-2 pb-2">Parent
          <select className="custom-select d-block"
                  disabled={this.props.parents.length == 0}
                  ref={this.selectRef}
                  defaultValue={ (this.props.parents.length > 0) ? this.props.parents[0].name : null }>
            {
              this.props.parents
                .map(x => (<option key={x.name} value={x.name}>{x.name}</option>))
            }
          </select>
        </label>
        <EditableNode ros={this.props.ros}
                      bt_namespace={this.props.bt_namespace}
                      key={this.props.node.module
                           + this.props.node.node_class
                           + this.props.node.name}
                      name={this.state.name}
                      nodeClass={this.props.node.node_class}
                      module={this.props.node.module}
                      doc={this.props.node.doc}
                      changeCopyMode={this.props.changeCopyMode}
                      messagesFuse={this.props.messagesFuse}
                      updateValidity={this.updateValidity}
                      updateValue={this.updateValue}
                      nameChangeHandler={this.nameChangeHandler}
                      options={this.state.options}
                      inputs={this.state.inputs}
                      outputs={this.state.outputs}
                      option_wirings={this.props.node.option_wirings}
        />
      </div>
    );
  }

  updateValidity(newValidity)
  {
    this.setState({isValid: newValidity || false});
  }

  onClickAdd()
  {
    this.props.onNodeChanged(false);
    var msg = this.buildNodeMessage();
    console.log('trying to add node to tree:');
    console.log(msg);
    this.add_node_service.callService(
      new ROSLIB.ServiceRequest({
        parent_name: this.selectRef.current.value || '',
        node: msg,
        allow_rename: true
      }),
      function(response) {
        if (response.success) {
          console.log('Added node to tree as ' + response.actual_node_name);
        }
        else {
          this.props.onError('Failed to add node ' + this.state.name + ': '
                      + response.error_message);
        }
      }.bind(this));
  }

  buildNodeMessage()
  {
    return {
      module: this.props.node.module,
      node_class: this.props.node.node_class,
      name: this.state.name,
      options: this.state.options.map(x => {
        var option = {
          key: x.key,
          serialized_value: ''
        };
        if (x.value.type === 'type') {
          if (python_builtin_types.indexOf(x.value.value) >= 0)
          {
            x.value.value = '__builtin__.' + x.value.value;
          }
          option.serialized_value = JSON.stringify({
            "py/type": x.value.value
          });
        }
        else if (x.value.type.startsWith('__'))
        {
          x.value.value["py/object"] = x.value.type.substring('__'.length);
          option.serialized_value = JSON.stringify(x.value.value);
        }
        else
        {
          option.serialized_value = JSON.stringify(x.value.value);
        }
        return option;
      }),
      child_names: []
    };
  }

  nameChangeHandler(event)
  {
    this.props.onNodeChanged(true);
    this.setState({name: event.target.value});
  }

  updateValue(paramType, key, new_value)
  {
    this.props.onNodeChanged(true);
    var map_fun = function(x)
    {
      if (x.key === key) {
        return {
          key: key,
          value: {
            type: x.value.type,
            value: new_value
          }
        };
      }
      else
      {
        return x;
      }
    };

    if (paramType.toLowerCase() === 'options')
    {
      var ref_keys = this.props.node.options
          .filter(x => prettyprint_type(x.serialized_value).startsWith('OptionRef('))
          .map(x => [x.key, prettyprint_type(x.serialized_value).substring(
            'OptionRef('.length, prettyprint_type(x.serialized_value).length - 1)])
          .filter(x => x[1] === key);
      this.setState(
        (prevState, props) =>
          {
            var new_options = prevState.options.map(map_fun);
            // update the options in our state that are references to
            // the changed key - this will discard any values entered
            // already, but they'd be incompatible anyway

            var resolved_options = new_options.map(x => {
              var refData = ref_keys.find(ref => ref[0] === x.key);
              if (refData)
              {
                var optionType = new_options.find(opt => opt.key === refData[1]);
                if (optionType)
                {
                  return {
                    key: x.key,
                    value: getDefaultValue(optionType.value.value.replace('__builtin__.', '').replace('builtins.', ''))
                  };
                }
              }
              return x;
            });
            return {options: resolved_options};
          });
    }
    else if (paramType.toLowerCase() === 'inputs')
    {
      this.setState(
        (prevState, props) =>
          {
            return {inputs: prevState.inputs.map(map_fun)};
          });
    }
    else if (paramType.toLowerCase() === 'outputs')
    {
      this.setState(
        (prevState, props) =>
          {
            return {outputs: prevState.outputs.map(map_fun)};
          });
    }
  }

  getDefaultValues(paramList, options)
  {
    options = options || [];

    return paramList.map(x => {
      return {
        key: x.key,
        value: getDefaultValue(
          prettyprint_type(x.serialized_value), options),
      };
    });
  }
}

class SelectedNode extends Component
{
  constructor(props)
  {
    super(props);

    var getValues = x =>
      {
        var type = prettyprint_type(x.serialized_type);
        var json_value = JSON.parse(x.serialized_value);
        if (type === 'type')
        {
          json_value = json_value['py/type']
            .replace('__builtin__.', '').replace('builtins.', '');
        }
        return {
          key: x.key,
          value: {type: type
                  .replace(/^basestring$/, 'string')
                  .replace(/^str$/, 'string')
                  .replace(/^unicode$/, 'string'),
                  value: json_value}
        };
      };
    if (props.node) {
      this.state = {
        name: props.node.name,
        node_class: props.node.node_class,
        module: props.node.module,
        isValid: true,
        options: props.node.options.map(getValues),
        inputs: props.node.inputs.map(getValues),
        outputs: props.node.outputs.map(getValues),
        isMorphed: false
      };
    }
    else
    {
      this.state = {
        name: '',
        node_class: '',
        module: '',
        isValid: false,
        options: [],
        inputs: [],
        outputs: [],
        isMorphed: false
      };
    }

    this.nameChangeHandler = this.nameChangeHandler.bind(this);
    this.nodeClassChangeHandler = this.nodeClassChangeHandler.bind(this);
    this.updateValidity = this.updateValidity.bind(this);
    this.updateValue = this.updateValue.bind(this);
    this.onClickDelete = this.onClickDelete.bind(this);
    this.onClickDeleteWithChildren = this.onClickDeleteWithChildren.bind(this);
    this.onClickUpdate = this.onClickUpdate.bind(this);
    this.updateNode = this.updateNode.bind(this);
  }

  componentDidMount()
  {
    this.set_options_service = new ROSLIB.Service({
      ros: this.props.ros,
      name: this.props.bt_namespace + 'set_options',
      serviceType: 'ros_bt_py_msgs/SetOptions'
    });

    this.remove_node_service = new ROSLIB.Service({
      ros: this.props.ros,
      name: this.props.bt_namespace + 'remove_node',
      serviceType: 'ros_bt_py_msgs/RemoveNode'
    });

    this.morph_node_service = new ROSLIB.Service({
      ros: this.props.ros,
      name: this.props.bt_namespace + 'morph_node',
      serviceType: 'ros_bt_py_msgs/MorphNode'
    });
  }

  nameChangeHandler(event)
  {
    this.props.onNodeChanged(true);
    this.setState({name: event.target.value});
  }

  nodeClassChangeHandler(event)
  {
    var flowControlNode = this.props.availableNodes.filter(function(item){
      return item.max_children == -1 && item.module+item.node_class == event.target.value;
    });

    if (flowControlNode && flowControlNode.length == 1) {
      flowControlNode = flowControlNode[0];

      this.setState({node_class: flowControlNode.node_class,
                     module: flowControlNode.module,
                     options: this.getDefaultValues(flowControlNode.options),
                     inputs: this.getDefaultValues(flowControlNode.inputs,
                                                   flowControlNode.options),
                     outputs: this.getDefaultValues(flowControlNode.outputs,
                                                    flowControlNode.options),
                     isMorphed: true});
    }
  }

  buildNodeMessage()
  {
    return {
      module: this.state.module,
      node_class: this.state.node_class,
      name: this.props.node.name,
      options: this.state.options.map(x => {
        var option = {
          key: x.key,
          serialized_value: ''
        };
        if (x.value.type === 'type') {
          if (python_builtin_types.indexOf(x.value.value) >= 0)
          {
            x.value.value = '__builtin__.' + x.value.value;
            //x.value.value = 'builtins.' + x.value.value;
          }
          option.serialized_value = JSON.stringify({
            "py/type": x.value.value
          });
        }
        else if (x.value.type.startsWith('__'))
        {
          x.value.value["py/object"] = x.value.type.substring('__'.length);
          option.serialized_value = JSON.stringify(x.value.value);
        }
        else
        {
          option.serialized_value = JSON.stringify(x.value.value);
        }
        return option;
      }),
      child_names: []
    };
  }

  getDefaultValues(paramList, options)
  {
    options = options || [];

    return paramList.map(x => {
      return {
        key: x.key,
        value: getDefaultValue(
          prettyprint_type(x.serialized_value), options),
      };
    });
  }

  onClickDelete(event)
  {
    if (!window.confirm("Really delete node " + this.props.node.name + "?"))
    {
      // Do nothing if user doesn't confirm
      return;
    }
    this.remove_node_service.callService(
      new ROSLIB.ServiceRequest({
        node_name: this.props.node.name,
        remove_children: false
      }),
      function(response) {
        if (response.success) {
          console.log('Removed node!');
          this.props.onEditorSelectionChange(null);
        }
        else {
          this.props.onError('Failed to remove node '
                             + this.props.node.name + ': '
                             + response.error_message);
        }
      }.bind(this));
  }

  onClickDeleteWithChildren(event)
  {
    if (!window.confirm("Really delete node "
                        + this.props.node.name
                        + " and all of its children?"))
    {
      // Do nothing if user doesn't confirm
      return;
    }
    this.remove_node_service.callService(
      new ROSLIB.ServiceRequest({
        node_name: this.props.node.name,
        remove_children: true
      }),
      function(response) {
        if (response.success) {
          console.log('Removed node!');
          this.props.onEditorSelectionChange(null);
        }
        else {
          this.props.onError('Failed to remove node '
                             + this.props.node.name + ': '
                             + response.error_message);
        }
      }.bind(this));
  }

  onClickUpdate(event)
  {
    if (this.state.isMorphed)
    {
      console.log("morphing node");
      var msg = this.buildNodeMessage();
      this.morph_node_service.callService(
        new ROSLIB.ServiceRequest({
          node_name: this.props.node.name,
          new_node: msg,
        }),
        function(response) {
          if (response.success) {
            console.log('Morphed node in tree');
            this.setState({isMorphed: false});
            this.updateNode();
          }
          else {
            this.props.onError('Failed to morph node ' + this.state.name + ': '
                        + response.error_message);
          }
        }.bind(this));
    } else {
      this.updateNode();
    }
  }

  updateNode()
  {
    console.log('updating node');
    this.set_options_service.callService(
      new ROSLIB.ServiceRequest({
        node_name: this.props.node.name,
        rename_node: true,
        new_name: this.state.name,
        options: this.state.options.map(x => {
          var option = {
            key: x.key,
            serialized_value: ''
          };
          if (x.value.type === 'type') {
            if (python_builtin_types.indexOf(x.value.value) >= 0)
            {
              x.value.value = '__builtin__.' + x.value.value;
            }
            option.serialized_value = JSON.stringify({
              "py/type": x.value.value
            });
          }
          else if (x.value.type.startsWith('__'))
          {
            x.value.value["py/object"] = x.value.type.substring('__'.length);
            option.serialized_value = JSON.stringify(x.value.value);
          }
          else
          {
            option.serialized_value = JSON.stringify(x.value.value);
          }
          return option;
        })
      }),
      function(response) {
        if (response.success) {
          console.log('Updated node!');
          this.props.onNodeChanged(false);
          this.props.onEditorSelectionChange(this.state.name); // FIXME: is there a more elegant way for the "update" case?
        }
        else {
          this.props.onError('Failed to update node '
                             + this.props.node.name + ': '
                             + response.error_message);
        }
      }.bind(this));
  }

  render()
  {
    return (
      <div className="d-flex flex-column">
        <div className="btn-group d-flex mb-2" role="group">
          <button className="btn btn-primary w-30"
                  disabled={!this.state.isValid}
                  onClick={this.onClickUpdate}>
            Update Node
          </button>
          <button className="btn btn-danger w-35"
                  onClick={this.onClickDelete}>
            Delete Node
          </button>
          <button className="btn btn-danger w-35"
                  onClick={this.onClickDeleteWithChildren}>
            Delete Node + Children
          </button>
        </div>
        <EditableNode ros={this.props.ros}
                      bt_namespace={this.props.bt_namespace}
                      key={this.props.node.module
                           + this.props.node.node_class
                           + this.props.node.name}
                      name={this.state.name}
                      nodeClass={this.state.node_class}
                      module={this.state.module}
                      availableNodes={this.props.availableNodes}
                      changeCopyMode={this.props.changeCopyMode}
                      messagesFuse={this.props.messagesFuse}
                      updateValidity={this.updateValidity}
                      updateValue={this.updateValue}
                      nameChangeHandler={this.nameChangeHandler}
                      nodeClassChangeHandler={this.nodeClassChangeHandler}
                      options={this.state.options}
                      inputs={this.state.inputs}
                      outputs={this.state.outputs}
                      option_wirings={this.props.node.option_wirings}
        />
      </div>
    );
  }

  updateValidity(newValidity)
  {
    this.props.onNodeChanged(true);
    this.setState({isValid: newValidity || false});
  }

  updateValue(paramType, key, new_value)
  {
    this.props.onNodeChanged(true);
    var map_fun = function(x)
    {
      if (x.key === key) {
        return {
          key: key,
          value: {
            type: x.value.type,
            value: new_value
          }
        };
      }
      else
      {
        return x;
      }
    };

    if (paramType.toLowerCase() === 'options')
    {
      // All of these are lists containing lists of [key, ref_key]
      //
      // That is, if options = { foo : int, bar : OptionRef(foo) }
      // ref_keys will be [[bar, foo]]
      var ref_keys = this.props.nodeInfo.options
          .filter(x => prettyprint_type(x.serialized_value).startsWith('OptionRef('))
          .map(x => [x.key, prettyprint_type(x.serialized_value).substring(
            'OptionRef('.length, prettyprint_type(x.serialized_value).length - 1)])
          .filter(x => x[1] === key);
      var input_option_ref_keys = this.props.nodeInfo.inputs
          .filter(x => prettyprint_type(x.serialized_value).startsWith('OptionRef('))
          .map(x => [x.key, prettyprint_type(x.serialized_value).substring(
            'OptionRef('.length, prettyprint_type(x.serialized_value).length - 1)])
          .filter(x => x[1] === key);
      var output_option_ref_keys = this.props.nodeInfo.inputs
          .filter(x => prettyprint_type(x.serialized_value).startsWith('OptionRef('))
          .map(x => [x.key, prettyprint_type(x.serialized_value).substring(
            'OptionRef('.length, prettyprint_type(x.serialized_value).length - 1)])
          .filter(x => x[1] === key);
      this.setState(
        (prevState, props) =>
          {
            // Replace the option value
            var new_options = prevState.options.map(map_fun);

            // update the options in our state that are references to
            // the changed key - this will discard any values entered
            // already, but they'd be incompatible anyway

            var resolve_refs = current_item => {
              // See if the current option references the changed key
              var refData = ref_keys.find(ref => ref[0] === current_item.key);
              if (refData)
              {
                // If it does, find the type of the referred key
                var optionType = new_options.find(opt => opt.key === refData[1]);
                if (optionType)
                {
                  // Get a default value for the type indicated by the
                  // referenced option
                  return {
                    key: current_item.key,
                    value: getDefaultValue(optionType.value.value.replace('__builtin__.', '').replace('builtins.', ''))
                  };
                }
              }
              return current_item;
            };

            var resolved_options = new_options.map(resolve_refs);
            var newState = {options: resolved_options};
            // if there's inputs or outputs that reference the changed
            // option key, update them too (this is just for display
            // and won't change anything in the backend)
            if (input_option_ref_keys.length > 0)
            {
              newState.inputs = prevState.inputs.map(resolve_refs);
            }
            if (output_option_ref_keys.length > 0)
            {
              newState.outputs = prevState.outputs.map(resolve_refs);
            }
            return newState;
          });
    }
    else if (paramType.toLowerCase() === 'inputs')
    {
      this.setState(
        (prevState, props) =>
          {
            return {inputs: prevState.inputs.map(map_fun)};
          });
    }
    else if (paramType.toLowerCase() === 'outputs')
    {
      this.setState(
        (prevState, props) =>
          {
            return {outputs: prevState.outputs.map(map_fun)};
          });
    }
  }
}

class EditableNode extends Component
{
  constructor(props)
  {
    super(props);
    this.state = {messages_results:[], results_at_key: null, selected_message: null};

    this.onFocus = this.onFocus.bind(this);

    this.jsonRef = React.createRef();

    this.handleOptionWirings = this.handleOptionWirings.bind(this);
    this.selectMessageResult = this.selectMessageResult.bind(this);

    this.get_message_fields_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'get_message_fields',
      serviceType: 'ros_bt_py_msgs/GetMessageFields'
    });
  }

  selectMessageResult(result)
  {
    this.setState({messages_results:[], selected_message: result});
  }

  renderSearchResults(results, key, onNewValue)
  {    
    if(results.length > 0 && this.state.results_at_key === key)
    {
      var result_rows = results.map(x => {
        return (
          <div className="list-group-item search-result"
               onClick={ () => { this.selectMessageResult(x); onNewValue(x.msg);}}>
            {x.msg}
          </div>
        );
      });

      return (
        <div className="mb-2 search-results" ref={node => this.node = node}>
          <div className="list-group">
              {result_rows}
          </div>
        </div>
      );
    } else {
      return null;
    }
  }

  componentDidMount()
  {
    document.addEventListener('click', this.handleClick);
  }

  handleClick = (event) => {
    if(this.node && !this.node.contains(event.target))
    {
      this.setState({messages_results:[]});
    }
  };

  onFocus(event)
  {
    this.props.changeCopyMode(false);
  }

  render()
  {
    var compareKeys = function(a, b) {
      if (a.key === b.key) {
        return 0;
      }
      if (a.key < b.key) {
        return -1;
      }
      return 1;
    };

    var node_class_name = this.props.nodeClass;
    if (this.props.availableNodes != null) {
      // get the flow control nodes
      var flowControlNodes = this.props.availableNodes.filter(function(item){
        return item.max_children == -1;
      });

      var module = this.props.module;
      var node_class = this.props.nodeClass;

      var currentFlowControlType = flowControlNodes.filter(function(item){
        return item.module == module && item.node_class == node_class;
      });

      if (currentFlowControlType.length > 0)
      {
        node_class_name = (<select className="custom-select"
                  value={this.props.module+this.props.nodeClass}
                  onChange={this.props.nodeClassChangeHandler}>
              {
                flowControlNodes.map(
                  x => (<option key={x.module+x.node_class}
                                value={x.module+x.node_class}>{x.node_class} ({x.module})</option>))
              }
          </select>);
      }
    }

    var doc_icon = null;
    if (this.props.doc)
    {
      doc_icon = (<i title={getShortDoc(this.props.doc)} class="fas fa-question-circle pl-2 pr-2"></i>);
    }

    return(
      <div className="d-flex flex-column">
        <input className="form-control-lg mb-2"
               type="text"
               value={this.props.name}
               onFocus={this.onFocus}
               onChange={this.props.nameChangeHandler}/>
        <div className="d-flex minw0">
          <h4 className="text-muted">{node_class_name}</h4>
          {doc_icon}
        </div>
        {this.renderParamInputs(this.props.options.sort(compareKeys), 'options')}
        {this.renderParamDisplays(this.props.inputs.sort(compareKeys), 'inputs')}
        {this.renderParamDisplays(this.props.outputs.sort(compareKeys), 'outputs')}
      </div>
    );
  }

  handleOptionWirings(paramType, key, new_value)
  {
    if (this.props.option_wirings)
    {
      this.props.option_wirings.forEach(function(option_wiring) {
        if (option_wiring.source === key)
        {
          var referenced_option = this.props.options.filter(function(item){
            return item.key == option_wiring.source;
          });

          if (referenced_option && referenced_option.length > 0)
          {
            var message = getMessageType(new_value);
            this.get_message_fields_service.callService(
              new ROSLIB.ServiceRequest({
                message_type: message.message_type,
                service: message.service
              }),
              function(response) {
                if (response.success) {
                  var obj = JSON.parse(response.fields);
                  this.props.updateValue('options', option_wiring.target, obj);
                }
              }.bind(this));
          }
        }
      }.bind(this));
    }
  }

  updateValue(paramType, key, new_value)
  {
    if (paramType.toLowerCase() === 'options')
    {
      this.handleOptionWirings(paramType, key, new_value);
    }

    this.props.updateValue(paramType, key, new_value);
  }

  inputForValue(paramItem, onValidityChange, onNewValue)
  {
    var valueType = paramItem.value.type;
    var changeHandler = (event) => {};
    var keyPressHandler = (event) => {};

    if (valueType === 'int')
    {
      // Number input with integer increments
      changeHandler = (event) =>
          {
            var newValue = Math.round(event.target.value);
            if (isNaN(newValue))
            {
              newValue = 0;
            }
            onNewValue(newValue);
          };

      return (
        <div className="form-group">
          <label className="d-block">{paramItem.key}
            <input type="number" name="integer"
                   className="form-control"
                   onChange={changeHandler}
                   placeholder="integer"
                   step="1.0"
                   value={paramItem.value.value}>
            </input>
          </label>
        </div>
      );
    }
    if (valueType === 'float')
    {
      // Number input with free increments
      changeHandler = (event) =>
          {
            var newValue = parseFloat(event.target.value);
            if (isNaN(newValue))
            {
              newValue = 0;
            }
            onNewValue(newValue);
          };

      return (
        <div className="form-group">
          <label className="d-block">{paramItem.key}
            <input type="number" name="float"
                   step="any"
                   className="form-control"
                   onChange={changeHandler}
                   onFocus={this.onFocus}
                   placeholder="float"
                   value={paramItem.value.value}>
            </input>
          </label>
        </div>
      );
    }
    else if (valueType === 'string')
    {
      // Regular input
      changeHandler = (event) =>
        {
          onNewValue(event.target.value || '');
        };

      return (
        <div className="form-group">
          <label className="d-block">{paramItem.key}
            <input type="text"
                   className="form-control mt-2"
                   value={paramItem.value.value}
                   onFocus={this.onFocus}
                   onChange={changeHandler}/>
          </label>
        </div>
      );
    }
    else if (valueType === 'type')
    {
      // Regular input
      changeHandler = (event) =>
        {
          var newTypeName = event.target.value || '';
          if (this.props.messagesFuse)
          {
            var results = this.props.messagesFuse.search(newTypeName);
            this.setState({messages_results: results.slice(0,5), results_at_key: paramItem.key});
          }
          if (python_builtin_types.indexOf(newTypeName) >= 0)
          {
            onNewValue('__builtin__.' + newTypeName);
          }
          else
          {
            onNewValue(newTypeName);
          }
        };

      keyPressHandler = (event) =>
        {
          if(event.key === 'Enter') {
            this.setState({messages_results: []});
            this.setState({results_at_key: null});
          }
        };

      return (
        <div className="form-group">
          <label className="d-block">{paramItem.key}
            <input type="text"
                   className="form-control mt-2"
                   value={paramItem.value.value}
                   onChange={changeHandler}
                   onFocus={this.onFocus}
                   onKeyPress={keyPressHandler} />
          </label>
          {this.renderSearchResults(this.state.messages_results, paramItem.key, onNewValue)}
        </div>
      );
    }
    else if (valueType === 'bool')
    {
      // Checkbox
      changeHandler = (event) =>
        {
          onNewValue(event.target.checked || false);
        };

      var checkID = 'input_checkbox_' + uuid();
      return (
        <div className="custom-control custom-checkbox m-1">
          <input type="checkbox"
                 id={checkID}
                 className="custom-control-input"
                 checked={paramItem.value.value}
                 onFocus={this.onFocus}
                 onChange={changeHandler} />
          <label className="custom-control-label d-block"
                 htmlFor={checkID}>
            {paramItem.key}
        </label>

        </div>
      );
    }
    else if (valueType === 'unset_optionref')
    {
      return (
        <div className="form-group m-1">
          <label className="d-block">{paramItem.key}
            <input type="text"
                   className="form-control mt-2"
                   value={paramItem.value.value}
                   disabled={true}/>
          </label>
        </div>
      );
    }
    else if (valueType === 'list')
    {
      return (
        <div className="form-group">
          <label className="d-block">{paramItem.key}&nbsp;
            <JSONInput json={paramItem.value.value}
                       message_type={paramItem.value.type}
                       ros={this.props.ros}
                       bt_namespace={this.props.bt_namespace}
                       output="list"
                       onValidityChange={onValidityChange}
                       onFocus={this.onFocus}
                       onNewValue={onNewValue}/>
          </label>
        </div>
      );
    }
    else if (valueType === 'dict')
    {
      return (
        <div className="form-group">
          <label className="d-block">{paramItem.key}
            <JSONInput json={paramItem.value.value}
                       message_type={paramItem.value.type}
                       ros={this.props.ros}
                       bt_namespace={this.props.bt_namespace}
                       output="dict"
                       onValidityChange={onValidityChange}
                       onFocus={this.onFocus}
                       onNewValue={onNewValue}/>
          </label>
        </div>
      );
    }
    else if (valueType === 'collections.OrderedDict')
    {
      return (
        <div className="form-group">
          <label className="d-block">{paramItem.key}
            <JSONInput json={paramItem.value.value}
                       message_type={paramItem.value.type}
                       ros={this.props.ros}
                       bt_namespace={this.props.bt_namespace}
                       output="dict"
                       onValidityChange={onValidityChange}
                       onFocus={this.onFocus}
                       onNewValue={onNewValue}/>
          </label>
        </div>
      );
    }

    else  // if (valueType === 'object')
    {
      return (
        <div className="form-group">
          <label className="d-block">{paramItem.key}
            <JSONInput json={paramItem.value.value}
                       message_type={paramItem.value.type}
                       ros={this.props.ros}
                       bt_namespace={this.props.bt_namespace}
                       output="pickled"
                       onValidityChange={onValidityChange}
                       onFocus={this.onFocus}
                       onNewValue={onNewValue}/>
          </label>
        </div>
      );
    }
  }

  displayForValue(paramItem)
  {
    var valueType = paramItem.value.type;

    if (valueType === 'int'
        || valueType === 'float'
        || valueType ==='string')
    {
      // Number input with integer increments
      return (
        <Fragment>
          <h5>{paramItem.key} <span className="text-muted">(type: {valueType})</span></h5>
          <span>{paramItem.value.value}</span>
        </Fragment>
      );
    }
    else if (valueType === 'type')
    {
      return (
        <Fragment>
          <h5>{paramItem.key} <span className="text-muted">(type: {valueType})</span></h5>
          <pre>{paramItem.value.value}</pre>
        </Fragment>
      );
    }
    else if (valueType === 'boolean' || valueType === 'bool')
    {
      return (
        <Fragment>
          <h5>{paramItem.key} <span className="text-muted">(type: {valueType})</span></h5>
          <pre>{paramItem.value.value ? 'True' : 'False'}</pre>
        </Fragment>
      );
    }
    else if (valueType === 'unset_optionref')
    {
      return (
        <Fragment>
          <h5>{paramItem.key} <span className="text-muted">(type: {valueType})</span></h5>
          <pre className="text-muted">{paramItem.value.value}</pre>
        </Fragment>
      );
    }
    // TODO(nberg): implement these two

    // else if (valueType === 'list')
    // {

    // }
    // else if (valueType === 'dict')
    // {

    // }
    else  // if (valueType === 'object')
    {
      console.log('item with non-basic type: ', paramItem);
      return (
        <Fragment>
          <h5>{paramItem.key} <span className="text-muted">(type: {valueType})</span></h5>
          <pre>{JSON.stringify(paramItem.value.value, null, 2)}</pre>
        </Fragment>
      );
    }
  }

  renderParamInputs(params, name)
  {
    var param_rows = params.map(x => {
      return (
        <div className="list-group-item"
             key={name + x.key}>
            {
              this.inputForValue(
                x,
                this.props.updateValidity,
                (newVal) => this.updateValue(name, x.key, newVal))
            }
        </div>
      );
    });

    return (
      <div className="mb-2">
        <h5>{name}</h5>
        <div className="list-group">
          {param_rows}
        </div>
      </div>
    );
  }

  renderParamDisplays(params, name)
  {
    var param_rows = params.map(x => {
      return (
        <div className="list-group-item"
             key={name + x.key}>
          {this.displayForValue(x)}
        </div>
      );
    });

    return (
      <div className="mb-2">
        <h5>{name}</h5>
        <div className="list-group">
            {param_rows}
        </div>
      </div>
    );
  }

  getDefaultValues(paramList, options)
  {
    options = options || [];

    return paramList.map(x => {
      return {
        key: x.key,
        value: getDefaultValue(
          prettyprint_type(x.serialized_value), options),
      };
    });
  }
}

class BehaviorTreeEdge extends Component
{
  constructor(props)
  {
    super(props);

    this.onClickDelete = this.onClickDelete.bind(this);
  }

  componentDidMount()
  {
    this.unwireClient = new ROSLIB.Service({
      ros: this.props.ros,
      name: this.props.bt_namespace + 'unwire_data',
      serviceType: 'ros_bt_py_msgs/WireNodeData'
    });
  }

  onClickDelete()
  {
    this.props.unsetSelectedEdge();
    this.unwireClient.callService(
      new ROSLIB.ServiceRequest({
        wirings: [{
          source: this.props.edge.source,
          target: this.props.edge.target
        }]
      }),
      function(response) {
        if (!response.success) {
          this.props.onError(response.error_message);
        }
      });
  }

  render()
  {
    return(
      <div className="d-flex flex-column">
        <div className="btn-group d-flex mb-2" role="group">
          <button className="btn btn-danger w-100"
                  onClick={this.onClickDelete}>Delete Edge</button>
        </div>
        <div className="row">
          <div className="col">
            <a href="#"
               className="text-primary"
               onClick={()=>this.props.onSelectionChange(this.props.edge.source.node_name)}>
              {this.props.edge.source.node_name}.
            </a>
            <span>{this.props.edge.source.data_kind}.</span>
            <span>{this.props.edge.source.data_key}</span>
          </div>
          <div className="col">
            <span aria-hidden="true" className="fas fa-lg fa-long-arrow-alt-right" />
            <span className="sr-only">is connected to</span>
          </div>
           <div className="col">
             <a href="#"
                className="text-primary"
                onClick={()=>this.props.onSelectionChange(this.props.edge.target.node_name)}>
              {this.props.edge.target.node_name}.
            </a>
            <span>{this.props.edge.target.data_kind}.</span>
            <span>{this.props.edge.target.data_key}</span>
          </div>
        </div>
      </div>
    );
  }
}



class JSONInput extends Component
{
  constructor(props)
  {
    super(props);

    var is_valid = false;

    try {
      JSON.parse(JSON.stringify(props.json));
      is_valid = true;
    }
    catch (e)
    {
      is_valid = false;
    }

    this.state = {
      is_valid: is_valid,
      json: props.json,
      message_type: props.message_type,
      field_names: [],
      pyobject: null
    };

    this.editor = null;
    this.editorRef = null;

    this.get_message_fields_service = new ROSLIB.Service({
      ros: props.ros,
      name: props.bt_namespace + 'get_message_fields',
      serviceType: 'ros_bt_py_msgs/GetMessageFields'
    });

    this.updateMessageType = this.updateMessageType.bind(this);
  }

  componentDidMount()
  {
    this.editor = new JSONEditor(this.editorRef, {
      mode: "code",
      onChange : this.handleChange,
    });
    this.editor.set(this.state.json);
    this.editor.aceEditor.setOptions({maxLines: 100});
    this.editor.aceEditor.resize();

    if (this.state.message_type != null)
    {
      this.updateMessageType(this.state.message_type, this.state.json, true);
    }
  }

  getJSONfromPyObject(pyobject, field_names)
  {
    var json = {};
    if (pyobject.hasOwnProperty("py/state"))
    {
      var counter = 0;
      for (var i = 0; i < pyobject["py/state"].length; i++) {
        var value = pyobject["py/state"][i];
        if (typeof value === 'object' && !Array.isArray(value))
        {
          var response = this.getJSONfromPyObject(value, field_names.slice(counter+1));
          json[field_names[counter]] = response.json;
          counter += response.counter + 1;
        } else {
          json[field_names[counter]] = value;
          counter += 1;
        }
      }
    }
    return {json:json, counter:counter};
  }

  getPyObjectFromJSON(pyobject, json)
  {
    var keys = Object.keys(json);
    if (pyobject.hasOwnProperty("py/state"))
    {
      for (var i = 0; i < pyobject["py/state"].length; i++) {
        var value = pyobject["py/state"][i];
        if (typeof value === 'object' && !Array.isArray(value))
        {
          pyobject["py/state"][i] = this.getPyObjectFromJSON(pyobject["py/state"][i], json[keys[i]]);
        } else {
          pyobject["py/state"][i] = json[keys[i]];
        }
      }
    }
    return pyobject;
  }

  updateMessageType(message_type, json, just_mounted)
  {
    var type_changed = true;
    if (this.state.message_type === message_type)
    {
      type_changed = false;
    } else {
      this.setState({message_type:message_type});
      type_changed = true;
    }
    if (this.props.ros)
    {
      var message = getMessageType(message_type);
      if (message.message_type == "/dict")
      {
        console.log("message is a dict, no request possible");
      } else {
        this.get_message_fields_service.callService(
          new ROSLIB.ServiceRequest({
            message_type: message.message_type,
            service: message.service
          }),
          function(response) {
            if (response.success) {
              this.setState({pyobject: response.fields});

              var new_value;
              if (type_changed) {
                new_value = this.getJSONfromPyObject(JSON.parse(response.fields), response.field_names).json;
              } else {
                new_value = this.getJSONfromPyObject(json, response.field_names).json;
              }

              this.setState({
                json : new_value,
                field_names: response.field_names
              });
              this.editor.update(new_value);
              console.log("updated message type and representation");
              if (!just_mounted) {
                this.handleChange();
              }
            }
          }.bind(this));
      }
    }
  }

  componentWillUnmount() {
    this.editor.destroy();
  }

  componentDidUpdate(prevProps, prevState) {
    if (JSON.stringify(this.props.json) != JSON.stringify(prevProps.json)) {
      if (this.props.json && this.props.json.hasOwnProperty("py/state"))
      {
        if (this.props.json.hasOwnProperty("py/object") && this.props.json["py/object"] != this.state.message_type)
        {
          this.updateMessageType(this.props.json["py/object"], this.props.json, false);
        }
      }
    }
  }

  handleChange = () => {
    try {
      var json = this.editor.get();
      this.setState({
        json : JSON.stringify(json),
        is_valid: true,
      });
      if (this.props.output == "pickled")
      {
        var reconstructed = this.getPyObjectFromJSON(JSON.parse(this.state.pyobject), json);
        this.props.onNewValue(reconstructed);
      } else {
        this.props.onNewValue(json);
      }
      
      this.props.onValidityChange(true);
    } catch (e) {
      this.setState({is_valid: false});
      this.props.onValidityChange(false);
    }
  };

  render()
  {
    return (
      <div id="editor"
           ref={(ref) => { this.editorRef = ref; }}
           onFocus={this.props.onFocus}
      />
    );
  }
}
