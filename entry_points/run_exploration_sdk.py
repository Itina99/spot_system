"""
setup sdk, crea adapters sdk e lancia exploration main loop
"""
import os
import sys
import bosdyn.client

from spot import spot_log_in_utils
from spot.adapters import SDKLocalGridProvider, SDKStateProvider, SDKMovementProvider, SDKVisualizerProvider, SDKRecordingProvider
from Algorithms.easy_walk import exploration_main_loop
from types import SimpleNamespace

import spot.spot_log_in_utils



def setup_spot(options):
    robot, lease_client, robot_state_client, client_metadata = spot_log_in_utils.setLogInfo(options)


def main():
    options = SimpleNamespace()
    options.name = "easyWalk"
    options.hostname = "192.168.80.3"
    options.verbose = False
    options.recording_user_name = ""
    options.recording_session_name = ""
    options.download_filepath = os.getcwd()

    try:

        return True
    except Exception as exc:
        logger = bosdyn.client.util.get_logger()
        logger.error('Spot threw an exception: %r', exc)
        return False

