# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import sys
import os
import logging
from ament_index_python import get_package_share_directory

logger = logging.getLogger(__name__)

# Patch for aioice, so each connection has the same username and password
import aioice

class Connection(aioice.Connection):
    local_username = aioice.utils.random_string(4)
    local_password = aioice.utils.random_string(22)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.local_username = Connection.local_username
        self.local_password = Connection.local_password

aioice.Connection = Connection  # type: ignore


import aiortc

if Version(aiortc.__version__) == Version("1.10.0"):
    X509_DIGEST_ALGORITHMS = {
        "sha-256": "SHA256",
    }
    aiortc.rtcdtlstransport.X509_DIGEST_ALGORITHMS = X509_DIGEST_ALGORITHMS

elif Version(aiortc.__version__) >= Version("1.11.0"):
    # Syntax changed in aiortc 1.11.0, so we need to use the hashes module
    from cryptography.hazmat.primitives import hashes

    X509_DIGEST_ALGORITHMS = {
        "sha-256": hashes.SHA256(),  # type: ignore
    }
    aiortc.rtcdtlstransport.X509_DIGEST_ALGORITHMS = X509_DIGEST_ALGORITHMS
