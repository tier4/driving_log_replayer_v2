# Copyright (c) 2022 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import xml.etree.ElementTree as ET
from importlib.metadata import version
from pathlib import Path


def _read_version() -> str:
    try:
        return version("driving-log-replayer-v2")
    except Exception:  # noqa: S110
        pass
    try:
        from ament_index_python.packages import get_package_prefix

        prefix = get_package_prefix("driving_log_replayer_v2_analyzer")
        pkg_xml = Path(prefix) / "share" / "driving_log_replayer_v2_analyzer" / "package.xml"
        if pkg_xml.is_file():
            root = ET.parse(pkg_xml).getroot()
            v = root.find("version")
            if v is not None and (t := (v.text or "").strip()):
                return t
    except Exception:  # noqa: S110
        pass
    return "0.0.0"


__version__ = _read_version()


def main() -> None:
    from . import __main__  # noqa
