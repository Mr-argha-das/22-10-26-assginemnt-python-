#!/usr/bin/env python3
"""Backward-compatible alias entry module re-exporting `WatchdogNode`."""

from .watchdog_node import WatchdogNode, main

__all__ = ['WatchdogNode', 'main']

if __name__ == '__main__':
    main()
