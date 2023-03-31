import importlib
import roslib


def load_message_type(message_type):
    # Get package and message name from the full message type
    package_name, message_name = message_type.split('/')

    # Load the message module
    roslib.load_manifest(package_name)
    message_module = importlib.import_module(package_name + '.msg')

    # Get the message class
    MessageType = getattr(message_module, message_name)
    return MessageType
