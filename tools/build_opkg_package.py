#!/usr/bin/env python3
"""
Build a package to be installed by opkg for use with CVRA package management.

See https://raymii.org/s/tutorials/Building_IPK_packages_by_hand.html for
reference.
"""

import argparse
import tempfile
import subprocess
import os
import shutil
import contextlib
import logging


@contextlib.contextmanager
def cd(path):
    cwd = os.getcwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(cwd)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="+", type=lambda s: tuple(s.split(":")))
    parser.add_argument("--name", "-p", help="Name of the package", required=True)
    parser.add_argument(
        "--maintainer",
        help="Maintainer of the package (name <email>)",
        default="CVRA <info@cvra.ch",
    )
    parser.add_argument("--description", help="Description of the package", default="")
    parser.add_argument(
        "--architecture",
        help="Architecture of the package, all by default",
        default="all",
    )
    parser.add_argument("--verbose", action="store_true", help="Enable debug output...")
    parser.add_argument("--output", "-o", help="Output package file", required=True)

    scripts = [
        ("postinst", "Post install script"),
        ("postrm", "Post uninstall script"),
        ("preinst", "Pre install script"),
        ("prerm", "Pre uninstall script"),
    ]
    for arg, help in scripts:
        parser.add_argument("--" + arg, help=help)

    args = parser.parse_args()

    for f in args.files:
        if len(f) != 2:
            parser.error("file format should be source:dst_in_package")

    return args


def build_control_archive(
    output_dir,
    pkg_name,
    pkg_description,
    pkg_maintainer,
    pkg_architecture,
    install_scripts,
):
    """
    Builds the archive containing the metadata for the package.

    install_scripts is a mapping from script names to source filenames.
    For example: {'preinst': 'test.sh'}. Allowed script names are: postinst,
    postrm, preinst and prerm.

    See https://www.debian.org/doc/debian-policy/ch-controlfields.html
    """
    ALLOWED_SCRIPTS = ["postinst", "postrm", "preinst", "prerm"]
    logging.info("Creating metadata archive...")
    metadata = {
        "Package": pkg_name,
        "Depends": "",
        "Maintainer": pkg_maintainer,
        "Description": pkg_description,
        "Architecture": pkg_architecture,
        "Version": "1.0.0",
    }
    files = ["control"]  # list of files to include in control archive
    with tempfile.TemporaryDirectory() as control_dir:
        with open(os.path.join(control_dir, "control"), "w") as f:
            for k, v in metadata.items():
                f.write("{}: {}\n".format(k, v))

        for dst, src in install_scripts.items():
            if dst not in ALLOWED_SCRIPTS:
                logging.warning("Unknown install script %s", dst)
            logging.debug("Copying %s to %s", src, dst)
            shutil.copy(src, os.path.join(control_dir, dst))
            files.append(dst)

        output = os.path.join(output_dir, "control.tar.gz")
        with cd(control_dir):
            archive(output, files)


def build_data_archive(output_dir, files):
    """
    Prepare the archive containing the actual data for the package.
    Files is a dict mapping file paths on the build machine to files in the
    installed machine, for example {'sshd.conf': '/etc/sshd/sshd.conf'}
    """
    logging.info("Creating data archive...")
    archive_files = []
    with tempfile.TemporaryDirectory() as data_dir:
        # Prepare the file definitions
        for src, dst in files.items():
            # remove leading /
            if dst.startswith("/"):
                dst = dst[1:]

            archive_files.append(dst)

            folder = os.path.dirname(dst)
            folder = os.path.join(data_dir, folder)
            logging.debug("Creating %s", folder)
            os.makedirs(folder, exist_ok=True)
            shutil.copy(src, os.path.join(data_dir, dst))

        output = os.path.join(output_dir, "data.tar.gz")

        with cd(data_dir):
            archive(output, archive_files)


def main():
    args = parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    scripts = {}

    if args.postinst:
        scripts["postinst"] = args.postinst
    if args.postrm:
        scripts["postrm"] = args.postrm
    if args.preinst:
        scripts["preinst"] = args.preinst
    if args.prerm:
        scripts["prerm"] = args.prerm

    with tempfile.TemporaryDirectory() as build_dir:
        # First build the package metadata
        build_control_archive(
            build_dir,
            args.name,
            args.description,
            args.maintainer,
            args.architecture,
            scripts,
        )

        # Then build the package content
        build_data_archive(build_dir, dict(args.files))

        # Put in the magic file to be detected as a package
        with open(os.path.join(build_dir, "debian-binary"), "w") as f:
            f.write("2.0")

        logging.info("Building final package...")
        with cd(build_dir):
            archive("package.ipk", ["debian-binary", "control.tar.gz", "data.tar.gz"])

        logging.info('Copying package to "%s"', args.output)
        shutil.move(os.path.join(build_dir, "package.ipk"), args.output)


def archive(output, files):
    """
    Compress the archive with the given files.
    """
    cmd = "tar --numeric-owner --group=0 --owner=0 -czf {} {}".format(
        output, " ".join(files)
    )
    logging.debug('About to run "%s"', cmd)
    subprocess.call(cmd.split())


if __name__ == "__main__":
    main()
