import theme

from nicegui import ui
import subprocess, netifaces

label_ip, label_wifi_name = None, None

#################################################
#################### PAGE #######################
#################################################

def create() -> None:
    @ui.page('/ip')
    def page_a():
        global label_ip, label_wifi_name
        with theme.frame('IP Page'):
            with ui.row():
                ui.label('IP: ').classes('text-h4 text-grey-8')
                label_ip = ui.label(get_ip_addresses()).classes('text-h4 text-dark-8')

            with ui.row():        
                ui.label('WiFi Network: ').classes('text-h4 text-grey-8')
                label_wifi_name = ui.label(get_wifi_name()).classes('text-h4 text-dark-8')

            ui.button("Update", on_click=update)


#################################################
#################### UTILS ######################
#################################################

def update():
    label_ip.set_text(get_ip_addresses())
    label_wifi_name.set_text(get_wifi_name())

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

    # return " ; ".join(ip_addresses)
    return ip_addresses[1].split(": ")[1]

def get_wifi_name():
    subprocess_result = subprocess.Popen('iwgetid',shell=True,stdout=subprocess.PIPE)
    subprocess_output = subprocess_result.communicate()[0],subprocess_result.returncode
    network_name = subprocess_output[0].decode('utf-8')
    return network_name.split(':')[1][1:-2] #1:-2 to remove the " character
