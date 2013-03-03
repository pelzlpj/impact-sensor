#!/usr/bin/env python2.7

from __future__ import print_function

import logging
import subprocess
import sys
import threading
import time


def get_rn42_address():
    """Returns: bluetooth address for the first Roving Networks device in range,
    or None if no devices were found.
    """
    try:
        devices = subprocess.check_output(('hcitool', 'scan'))
    except subprocess.CalledProcessError:
        return None
    for line in devices.split('\n'):
        if not line.startswith('\t'):
            continue
        try:
            (address, name) = line.strip().split('\t')
        except ValueError:
            sys.stderr.write('Warning: unexpected output from "hcitool scan"\n')
            return None
        if name.startswith('FireFly'):
            return address
    return None


class RN42Connection(object):

    def __init__(self, host_device):
        """Initiate a connection from the host device (e.g. 'hci0') to the first
        detected Roving Networks bluetooth device.
        """
        self._host_dev   = host_device
        self._agent_proc = subprocess.Popen(
                ('bluetooth-agent', '0000'))

        self._lock           = threading.Lock()
        self._exit_requested = False    # guarded by _lock
        self._proc           = None     # guarded by _lock
        self._rfcomm_dev     = None     # guarded by _lock

        self._thread = threading.Thread(target=self._rfcomm_thread)
        self._thread.start()

    def get_rfcomm_device(self):
        """Poll the rfcomm process to determine whether or not the connection
        state has changed.  The caller should be prepared for failure when opening
        the rfcomm device, as it may disappear at any time.

        Returns: rfcomm device associated with the connected RN42 (i.e. '/dev/rfcomm0'),
        or None if the connection is currently broken.
        """
        with self._lock:
            return self._rfcomm_dev

    def terminate(self):
        """Drop connection and clean up."""
        with self._lock:
            self._exit_requested = True
            if self._proc:
                self._proc.terminate()
        self._thread.join()
        self._agent_proc.terminate()
        self._agent_proc.communicate()

    def _rfcomm_thread(self):
        bluetooth_address = None
        logging.basicConfig(filename='rfcomm.log', level=logging.DEBUG)

        while True:
            with self._lock:
                if self._exit_requested:
                    logging.info('Normal exit.')
                    return
                if bluetooth_address and not self._proc:
                    # We need to read rfcomm stdout promptly in order to get
                    # realtime connection status.  'stdbuf' is used to turn off
                    # stdout buffering in the rfcomm process.
                    logging.info('Starting rfcomm.')
                    self._proc = subprocess.Popen(
                            ('stdbuf', '-o0', 'rfcomm', 'connect', self._host_dev,
                                bluetooth_address),
                            stdout=subprocess.PIPE)

            if not bluetooth_address:
                bluetooth_address = get_rn42_address()
                if bluetooth_address:
                    logging.info('RN42 located at address %s' % bluetooth_address)

            if self._proc:
                line = self._proc.stdout.readline()
                logging.info('rfcomm stdout: \"%s\"' % line.rstrip())
                if not line:
                    self._proc.wait()
                    logging.info('rfcomm exit code: %u' % self._proc.returncode)
                    with self._lock:
                        self._proc       = None
                        self._rfcomm_dev = None
                elif line.startswith('Connected '):
                    with self._lock:
                        self._rfcomm_dev = line.split(' ')[1]
                    logging.info('New rfcomm device: %s' % self._rfcomm_dev)



if __name__ == '__main__':
    conn = RN42Connection('hci0')
    old_dev = None
    try:
        while True:
            dev = conn.get_rfcomm_device()
            if dev and not old_dev:
                print('Connected: device %s' % dev)
            elif (not dev) and old_dev:
                print('No connection.')
            old_dev = dev
    finally:
        conn.terminate()


