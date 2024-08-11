import theme
from message import message

from nicegui import ui
import subprocess, netifaces

#################################################
#################### PAGE #######################
#################################################

def create() -> None:
    @ui.page('/ip')
    def page_a():
        with theme.frame('IP Page'):
            message('IP')
            with ui.element('div').classes('p-3 bg-blue-100'):
                label_ip = ui.label(get_ip_addresses())
            ui.timer(1.0, lambda: label_ip.set_text(get_ip_addresses()))
            message('WiFi Network')
            with ui.element('div').classes('p-3 bg-blue-100'):
                label_wifi_name = ui.label(get_wifi_name())
            ui.timer(1.0, lambda: label_wifi_name.set_text(get_wifi_name()))


#################################################
#################### UTILS ######################
#################################################

def get_ip_address(interface):
    try:
        addrs = netifaces.ifaddresses(interface)
        ip_info = {}
        if netifaces.AF_INET in addrs:  # Check for IPv4 address
            ip_info['IPv4'] = addrs[netifaces.AF_INET][0]['addr']
        if netifaces.AF_INET6 in addrs:  # Check for IPv6 address
            ip_info['IPv6'] = addrs[netifaces.AF_INET6][0]['addr']
        if not ip_info:
            return "Not connected"
        return ip_info['IPv4']
    except ValueError:
        return {}

def get_ip_addresses():
    interfaces = netifaces.interfaces()

    ip_addresses = []
    for interface_name in interfaces:
        ip_address = get_ip_address(interface_name)
        ip_addresses.append(f"{interface_name}: {ip_address}")

    return " ; ".join(ip_addresses)

def get_wifi_name():
    subprocess_result = subprocess.Popen('iwgetid',shell=True,stdout=subprocess.PIPE)
    subprocess_output = subprocess_result.communicate()[0],subprocess_result.returncode
    network_name = subprocess_output[0].decode('utf-8')
    return network_name

