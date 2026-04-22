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


from importlib.metadata import PackageNotFoundError
from importlib.metadata import version
from pathlib import Path
import xml.etree.ElementTree as ET

try:
    from ament_index_python.packages import get_package_prefix
except ImportError:
    get_package_prefix = None  # e.g. unit tests without ament_index_python


def _read_version() -> str:
    out = "0.0.0"
    try:
        out = version("driving-log-replayer-v2")
    except PackageNotFoundError:
        if get_package_prefix is not None:
            try:
                prefix = get_package_prefix("driving_log_replayer_v2_analyzer")
            except (KeyError, ValueError):
                pass
            else:
                pkg_xml = (
                    Path(prefix) / "share" / "driving_log_replayer_v2_analyzer" / "package.xml"
                )
                if pkg_xml.is_file():
                    try:
                        # Local package.xml from this package install; not untrusted network input
                        root = ET.parse(pkg_xml).getroot()  # noqa: S314
                    except ET.ParseError:
                        pass
                    else:
                        el = root.find("version")
                        if el is not None and (t := (el.text or "").strip()):
                            out = t
    return out


__version__ = _read_version()


def main() -> None:
    from . import __main__  # noqa
