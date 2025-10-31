# Copyright (c) 2025 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

from datetime import datetime
from datetime import timezone
import logging
from pathlib import Path


def custom_text_formatter() -> logging.Formatter:
    return logging.Formatter(
        "[%(asctime)s] [%(levelname)s] [%(filename)s:%(lineno)d %(funcName)s] %(message)s"
    )


class SensitiveWordFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        sensitive_words = [
            "password",
            "auth_token",
            "secret",
        ]
        log_message = record.getMessage()
        return all(word not in log_message for word in sensitive_words)


def configure_logger(
    log_file_directory: str,
    console_log_level: int = logging.INFO,
    file_log_level: int = logging.INFO,
    logger_name: str = "",
) -> logging.Logger:
    """
    Create and configure a logger.

    Args:
        log_file_directory (str): The directory path to save log.
        console_log_level (int): Log level for console. Defaults to logging.INFO.
        file_log_level (int): Log level for log file. Defaults to logging.INFO.
        logger_name (str): Name for logger. Defaults to "".

    """
    # make directory
    log_dir = Path(log_file_directory)
    log_dir.mkdir(parents=True, exist_ok=True)

    formatter = custom_text_formatter()

    logger = logging.getLogger(logger_name)
    logger.handlers.clear()
    logger.addFilter(SensitiveWordFilter())
    logger.setLevel(min(console_log_level, file_log_level))

    # handler for console
    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(console_log_level)
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

    # handler for file
    time_str = f"{datetime.now(timezone.utc):%Y%m%d_%H%M%S}.log"
    log_file_path = log_dir / time_str
    file_handler = logging.FileHandler(filename=str(log_file_path), encoding="utf-8")
    file_handler.setLevel(file_log_level)
    file_handler.setFormatter(custom_text_formatter())
    logger.addHandler(file_handler)

    return logger
