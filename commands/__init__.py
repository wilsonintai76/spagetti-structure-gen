# SpaghettiStructureGen — commands registry
# Template stubs (commandDialog, paletteShow, paletteSend) are left on disk
# but deregistered. Only the SpaghettiStructureGen command is active.

from .SpaghettiStructureGen import entry as SpaghettiStructureGen

commands = [
    SpaghettiStructureGen,
]


def start():
    for command in commands:
        command.start()


def stop():
    for command in commands:
        command.stop()