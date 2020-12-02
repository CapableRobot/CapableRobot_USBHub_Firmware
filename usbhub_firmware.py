#!/usr/bin/env python3

import os, sys, glob, time
import shutil

import click

def wait_for_file(path):
    while not os.path.exists(path):
        time.sleep(0.1)
    time.sleep(0.5)

def wait_for_circuitpy(drive):
    if "USBHUBBOOT" in drive:
        wait_for_file(drive.replace("USBHUBBOOT", "CIRCUITPY"))
    else:
        wait_for_file(os.path.join("{}/boot_out.txt".format(drive)))

def wait_for_bootloader(drive):
    if "CIRCUITPY" in drive:
        wait_for_file(drive.replace("CIRCUITPY", "USBHUBBOOT"))
    else:
        wait_for_file(os.path.join("{}/INFO_UF2.TXT".format(drive)))

def do_file_update(src_path, dst_path):
    
    if not os.path.exists(dst_path):
        return True

    if os.path.getmtime(src_path) > os.path.getmtime(dst_path):
        return True

    return False

def upgrade_circuitpython(drive, uf2_version, uf2_file):
    code_file = os.path.join(drive, "code.py")
    code_backup = "{}.bkp".format(code_file)
    print("... rebooting into CircuitPython Bootloader")
    os.rename(code_file, code_backup)

    with open(code_file, 'w') as handle:
        handle.write("import microcontroller\n")
        handle.write("import sys, time\n")
        handle.write("this_version = '.'.join([str(v) for v in sys.implementation.version])\n")
        handle.write("if this_version == '{}':\n".format(uf2_version))
        handle.write("    print('Circuitpython upgrade complete')\n")
        handle.write("    while True:\n")
        handle.write("        time.sleep(0.1)\n")
        handle.write("else:\n")
        handle.write("    print('Circuitpython will be upgraded')\n")
        handle.write("    microcontroller.on_next_reset(microcontroller.RunMode.BOOTLOADER)\n")
        handle.write("    microcontroller.reset()\n")

    wait_for_bootloader(drive)
    print("... installing CircuitPython via UF2 file")
    shutil.copy(uf2_file, drive.replace("CIRCUITPY", "USBHUBBOOT"))    

    wait_for_circuitpy(drive)
    print("... restoring code.py file")
    os.rename(code_backup, code_file)

@click.group()
def cli():
    pass

def file_to_version(path):
    return path.split("en_US-")[1].replace(".uf2","")

def check_for_usbhub(drive):
    if not os.path.exists(drive):
        print("Specified drive '{}' does not exist".format(drive))
        sys.exit(0)

    boot_file = os.path.join(drive, "boot_out.txt")

    if not os.path.exists(boot_file):
        print("Specified drive '{}' does not appear to be a CircuitPython device".format(drive))
        sys.exit(0)

    boot_file_string = open(boot_file, "r").read().strip().split(";")

    if boot_file_string[1].strip() != "Capable Robot Programmable USB Hub with samd51g19":
        print("Specified drive '{}' is not a Capable Robot Programmable USB Hub".format(drive))
        print("    it reports : {}".format(boot_file_string[1].strip()))
        sys.exit(0)

    return boot_file_string

@cli.command()
@click.option('--version', default=None, help='Version to install')
@click.argument('drive')
def circuitpython(drive, version):
    """ Reflash MCU with a different version of Circuitpython """

    boot_file_string = check_for_usbhub(drive)
    active_version = boot_file_string[0].split(" ")[2]

    if version is None:
        target_uf2 = sorted(glob.glob("./assets/adafruit*.uf2"))[-1]
    else:
        uf2s = glob.glob("./assets/adafruit*{}.uf2".format(version))

        if len(uf2s) == 0:
            versions = [file_to_version(f) for f in sorted(glob.glob("./assets/adafruit*.uf2"))]
            print("Specified version ({}) is not found".format(version))
            print("Please choose from : {}".format(" ".join(versions)))
            sys.exit(0)

        target_uf2 = sorted(uf2s)[-1]

    target_version = file_to_version(target_uf2)

    print("CircuitPython Version : {}".format(active_version))

    if active_version == target_version:
        print("    at specified release")
    else:
        print("    will be upgraded to version {}".format(target_version))
        upgrade_circuitpython(drive, target_version, target_uf2)
    

@cli.command()
@click.option('--main', default='code.py', help='Top-level code.py to write')
@click.option('--force', is_flag=True, help='Force file updates (ignores file modification times)')
@click.argument('drive')
def firmware(main, force, drive):
    """ Update MCU firmware """

    check_for_usbhub(drive)

    root_dir = os.path.dirname(os.path.realpath(__file__))

    for file in glob.glob("{}/lib/*.py".format(root_dir)) + glob.glob("{}/lib/**/*.py".format(root_dir)):
        src_path = file.replace(root_dir+"/","")
        dst_path = "{}/{}".format(drive, src_path)

        if do_file_update(file, dst_path) or force:
            
            dst_dir = os.path.dirname(dst_path)
            if not os.path.exists(dst_dir):
                print("Make dir : {}".format(dst_dir.replace(drive+"/","")))
                os.mkdir(dst_dir)

            print("Updating : {}".format(src_path))
            shutil.copy(file, dst_path)
        else:
            print("Skipping : {}".format(src_path))

    dst_path = "{}/code.py".format(drive)
    if do_file_update(main, dst_path) or force:
        print("Updating : code.py from {}".format(os.path.basename(main)))
        shutil.copy(main, "{}/code.py".format(drive))
    else:
        print("Skipping : code.py")

def main():
    cli()
    
if __name__ == '__main__':
    main()
