"""Read VectorView contact topics for closed-loop grasp control."""

from __future__ import annotations

import threading
import time

from gz.msgs10.contacts_pb2 import Contacts
from gz.transport13 import Node


class ContactMonitor:
    def __init__(self, left_topic: str, right_topic: str) -> None:
        self._node = Node()
        self._left_on_box = False
        self._right_on_box = False
        self._lock = threading.Lock()
        self._node.subscribe(Contacts, left_topic, self._on_left)
        self._node.subscribe(Contacts, right_topic, self._on_right)

    def _on_left(self, message: Contacts) -> None:
        with self._lock:
            self._left_on_box = self._touches_upper_box(message)

    def _on_right(self, message: Contacts) -> None:
        with self._lock:
            self._right_on_box = self._touches_upper_box(message)

    @staticmethod
    def _touches_upper_box(message: Contacts) -> bool:
        for contact in message.contact:
            if contact.collision2.name == "object::main::collision":
                return True
        return False

    @property
    def left_on_box(self) -> bool:
        with self._lock:
            return self._left_on_box

    @property
    def right_on_box(self) -> bool:
        with self._lock:
            return self._right_on_box

    def both_on_box(self) -> bool:
        return self.left_on_box and self.right_on_box

    def wait_for_both(self, timeout: float) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            if self.both_on_box():
                return True
            time.sleep(0.05)
        return False
