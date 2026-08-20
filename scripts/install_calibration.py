#!/usr/bin/env python3
"""Merge a PASS result into estimator/controller YAML with atomic backups."""

import argparse
import datetime
import os
import shutil
import sys
from pathlib import Path

import yaml

from vrpn_imu_cmd_calibration.install import (
    CONTROLLER_KEYS,
    ESTIMATOR_KEYS,
    merge_allowed,
    validate_asset,
)


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("%s must contain a YAML mapping" % path)
    return value


def atomic_install(path, value):
    path = Path(path).resolve()
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    backup = Path(str(path) + ".bak." + stamp)
    temporary = Path(str(path) + ".tmp")
    shutil.copy2(str(path), str(backup))
    temporary.write_text(
        yaml.safe_dump(value, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    os.replace(str(temporary), str(path))
    return backup


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_dir", help="directory containing calibration.yaml and generated YAML")
    parser.add_argument("--estimator-target", required=True)
    parser.add_argument("--controller-target")
    parser.add_argument("--apply", action="store_true", help="required to modify target files")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    result_dir = Path(args.result_dir).resolve()
    try:
        asset = load_yaml(result_dir / "calibration.yaml")
        validate_asset(asset)
        generated_estimator = load_yaml(result_dir / "estimator.yaml")
        current_estimator = load_yaml(args.estimator_target)
        merged_estimator = merge_allowed(current_estimator, generated_estimator, ESTIMATOR_KEYS)
        merged_controller = None
        if args.controller_target:
            generated_controller = load_yaml(result_dir / "controller.yaml")
            current_controller = load_yaml(args.controller_target)
            merged_controller = merge_allowed(
                current_controller, generated_controller, CONTROLLER_KEYS
            )
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print("refuse calibration install: %s" % exc, file=sys.stderr)
        return 2

    print("validated PASS asset: %s" % (result_dir / "calibration.yaml"))
    for key in ESTIMATOR_KEYS:
        print("estimator %s: %r -> %r" % (key, current_estimator.get(key), merged_estimator[key]))
    if merged_controller is not None:
        for key in CONTROLLER_KEYS:
            print("controller %s: %r -> %r" % (key, current_controller.get(key), merged_controller[key]))
    if not args.apply:
        print("dry run only; add --apply to write atomically with timestamped backups")
        return 0

    estimator_backup = atomic_install(args.estimator_target, merged_estimator)
    print("updated %s (backup %s)" % (args.estimator_target, estimator_backup))
    if merged_controller is not None:
        controller_backup = atomic_install(args.controller_target, merged_controller)
        print("updated %s (backup %s)" % (args.controller_target, controller_backup))
    return 0


if __name__ == "__main__":
    sys.exit(main())
