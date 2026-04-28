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
        if netifaces.AF_INET in addrs:
            return addrs[netifaces.AF_INET][0]['addr']
        return None
    except (ValueError, KeyError, IndexError):
        return None

def get_ip_addresses():
    try:
        for interface_name in netifaces.interfaces():
            if interface_name == 'lo':
                continue
            ip = get_ip_address(interface_name)
            if ip is not None:
                return ip
        return "No IP found"
    except Exception:
        return "Error retrieving IP"

def get_wifi_name():
    subprocess_result = subprocess.Popen('iwgetid',shell=True,stdout=subprocess.PIPE)
    subprocess_output = subprocess_result.communicate()[0],subprocess_result.returncode
    network_name = subprocess_output[0].decode('utf-8')
    return network_name.split(':')[1][1:-2] #1:-2 to remove the " character
# TODO réparer