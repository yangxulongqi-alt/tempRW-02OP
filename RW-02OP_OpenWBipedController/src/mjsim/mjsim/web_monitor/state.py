"""
System state monitoring utilities.
"""

import os
import time
from typing import Dict, Optional
from dataclasses import dataclass


@dataclass
class CPUStats:
    """CPU statistics."""
    usage_percent: float = 0.0
    load_avg_1m: float = 0.0
    load_avg_5m: float = 0.0
    load_avg_15m: float = 0.0
    cpu_count: int = 1


class SystemMonitor:
    """System resource monitor for CPU, memory, etc."""
    
    def __init__(self):
        self._prev_idle = 0
        self._prev_total = 0
        self._cpu_count = os.cpu_count() or 1
        self._last_update = 0
        self._cached_stats: Optional[CPUStats] = None
        self._update_interval = 0.5  # seconds
    
    def _read_cpu_times(self) -> tuple:
        """Read CPU times from /proc/stat."""
        try:
            with open('/proc/stat', 'r') as f:
                line = f.readline()
                parts = line.split()
                if parts[0] == 'cpu':
                    # user, nice, system, idle, iowait, irq, softirq, steal
                    times = [int(x) for x in parts[1:9]]
                    idle = times[3] + times[4]  # idle + iowait
                    total = sum(times)
                    return idle, total
        except (FileNotFoundError, PermissionError, ValueError):
            pass
        return 0, 0
    
    def get_cpu_usage(self) -> float:
        """
        Get current CPU usage percentage.
        
        Returns:
            CPU usage as percentage (0-100)
        """
        idle, total = self._read_cpu_times()
        
        if self._prev_total == 0:
            self._prev_idle = idle
            self._prev_total = total
            return 0.0
        
        idle_delta = idle - self._prev_idle
        total_delta = total - self._prev_total
        
        self._prev_idle = idle
        self._prev_total = total
        
        if total_delta == 0:
            return 0.0
        
        usage = 100.0 * (1.0 - idle_delta / total_delta)
        return max(0.0, min(100.0, usage))
    
    def get_load_average(self) -> tuple:
        """
        Get system load averages.
        
        Returns:
            Tuple of (1min, 5min, 15min) load averages
        """
        try:
            load = os.getloadavg()
            return load
        except (OSError, AttributeError):
            return (0.0, 0.0, 0.0)
    
    def get_cpu_stats(self, force_update: bool = False) -> CPUStats:
        """
        Get comprehensive CPU statistics.
        
        Args:
            force_update: Force refresh even if cache is fresh
            
        Returns:
            CPUStats dataclass with CPU information
        """
        now = time.time()
        
        if not force_update and self._cached_stats and \
           (now - self._last_update) < self._update_interval:
            return self._cached_stats
        
        usage = self.get_cpu_usage()
        load_1, load_5, load_15 = self.get_load_average()
        
        self._cached_stats = CPUStats(
            usage_percent=round(usage, 1),
            load_avg_1m=round(load_1, 2),
            load_avg_5m=round(load_5, 2),
            load_avg_15m=round(load_15, 2),
            cpu_count=self._cpu_count
        )
        self._last_update = now
        
        return self._cached_stats
    
    def get_memory_usage(self) -> Dict[str, float]:
        """
        Get memory usage statistics.
        
        Returns:
            Dict with memory info in MB
        """
        try:
            with open('/proc/meminfo', 'r') as f:
                meminfo = {}
                for line in f:
                    parts = line.split()
                    if len(parts) >= 2:
                        key = parts[0].rstrip(':')
                        value = int(parts[1]) / 1024  # Convert kB to MB
                        meminfo[key] = value
                
                total = meminfo.get('MemTotal', 0)
                free = meminfo.get('MemFree', 0)
                buffers = meminfo.get('Buffers', 0)
                cached = meminfo.get('Cached', 0)
                
                used = total - free - buffers - cached
                usage_percent = (used / total * 100) if total > 0 else 0
                
                return {
                    'total_mb': round(total, 1),
                    'used_mb': round(used, 1),
                    'free_mb': round(free + buffers + cached, 1),
                    'usage_percent': round(usage_percent, 1)
                }
        except (FileNotFoundError, PermissionError, ValueError):
            return {
                'total_mb': 0,
                'used_mb': 0,
                'free_mb': 0,
                'usage_percent': 0
            }


# Global singleton instance
_system_monitor: Optional[SystemMonitor] = None


def get_system_monitor() -> SystemMonitor:
    """Get the global SystemMonitor instance."""
    global _system_monitor
    if _system_monitor is None:
        _system_monitor = SystemMonitor()
    return _system_monitor


def get_cpu_usage() -> float:
    """
    Get current CPU usage percentage.
    
    Convenience function using the global monitor.
    
    Returns:
        CPU usage as percentage (0-100)
    """
    return get_system_monitor().get_cpu_usage()


def get_cpu_stats() -> CPUStats:
    """
    Get comprehensive CPU statistics.
    
    Convenience function using the global monitor.
    
    Returns:
        CPUStats dataclass with CPU information
    """
    return get_system_monitor().get_cpu_stats()


def get_memory_usage() -> Dict[str, float]:
    """
    Get memory usage statistics.
    
    Convenience function using the global monitor.
    
    Returns:
        Dict with memory info
    """
    return get_system_monitor().get_memory_usage()
