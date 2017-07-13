import uavcan
import os
import tempfile
import logging
import argparse
import sys

#logging.basicConfig(stream=sys.stderr, level=logging.ERROR,
                    #format='%(asctime)s %(levelname)s %(name)s %(message)s')

parser = argparse.ArgumentParser()
parser.add_argument('serve_file', nargs=1)
parser.add_argument('port', nargs=1)
parser.add_argument('node_name', nargs=1)
args = parser.parse_args()

port = args.port[0]
serve_file = args.serve_file[0]
update_node_name = args.node_name[0]
exit_timer = 5
existing_nodes = []
flashing_nodes = []
complete_nodes = []

def handle_firmwareupdare_resp(e):
    pass

def read_handler(e):
    resp = uavcan.protocol.file.Read.Response()
    resp.error.value = resp.error.UNKNOWN_ERROR

    if os.path.normpath(e.request.path.path.decode()) == os.path.normpath(serve_file):
        try:
            with open(serve_file) as f:
                exit_timer = 5
                f.seek(e.request.offset)
                read_size = uavcan.get_uavcan_data_type(uavcan.get_fields(resp)['data']).max_size
                resp.data = f.read(read_size)
                resp.error.value = resp.error.OK
                print("0")
                if len(resp.data) < read_size:
                    print("1")
                    if e.transfer.source_node_id in flashing_nodes:
                        print("2")
                        flashing_nodes.remove(e.transfer.source_node_id)
                        complete_nodes.append(e.transfer.source_node_id)

                    if len(flashing_nodes) == 0:
                        print("3")
                        exit_timer = 2
        except:
            pass
    return resp

def periodic_5hz():
    global exit_timer
    exit_timer -= 1
    if exit_timer == 0:
        if len(complete_nodes) > 0:
            print("flashed %u nodes" % (len(complete_nodes),), complete_nodes)

        if len(flashing_nodes) != 0:
            print("failed to flash nodes:", flashing_nodes)
            sys.exit(1)
        elif len(complete_nodes) == 0:
            print("failed to flash any nodes")
            sys.exit(1)
        else:
            sys.exit(0)

        print(existing_nodes, flashing_nodes)
        sys.exit()

def firmwareupdate_resp_handler(e):
    if e.response.error == e.response.ERROR_OK or e.response.error == e.response.ERROR_IN_PROGRESS:
        print("%u accepted firmwareupdate command" % (e.transfer.source_node_id,))

def getnodeinfo_resp_handler(e):
    if e.transfer.source_node_id not in existing_nodes:
        existing_nodes.append(e.transfer.source_node_id)
        if e.response.name.decode() == update_node_name:
            print("sending BeginFirmwareUpdate to %u" % (e.transfer.source_node_id,))
            node.request(uavcan.protocol.file.BeginFirmwareUpdate.Request(source_node_id=0, image_file_remote_path=uavcan.protocol.file.Path(path=serve_file)), e.transfer.source_node_id, firmwareupdate_resp_handler, priority=uavcan.TRANSFER_PRIORITY_LOWEST)

            flashing_nodes.append(e.transfer.source_node_id)


def node_status_handler(e):
    if e.transfer.source_node_id not in existing_nodes:
        node.request(uavcan.protocol.GetNodeInfo.Request(), e.transfer.source_node_id, getnodeinfo_resp_handler, priority=uavcan.TRANSFER_PRIORITY_LOWEST)

node = uavcan.make_node(port, node_id=126)
node_monitor = uavcan.app.node_monitor.NodeMonitor(node)
allocator = uavcan.app.dynamic_node_id.CentralizedServer(node, node_monitor, database_storage=tempfile.mktemp())

node.add_handler(uavcan.protocol.file.Read, read_handler)
node.add_handler(uavcan.protocol.NodeStatus, node_status_handler)
periodic_handle = node.periodic(0.2, periodic_5hz)



node.spin()
