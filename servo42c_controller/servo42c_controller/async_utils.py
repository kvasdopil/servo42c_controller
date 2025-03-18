"""Async utilities for the servo42c_controller package."""

import asyncio
import threading
from functools import wraps
import rclpy
from typing import Any, Callable, TypeVar

T = TypeVar('T')

class AsyncRunner:
    """Manages async operations in a separate thread."""
    
    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
    
    def _run_loop(self):
        """Run the event loop in a separate thread."""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()
    
    def run_coroutine(self, coro: Callable[..., T]) -> T:
        """Run a coroutine in the event loop and return its result."""
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        try:
            return future.result()
        except Exception as e:
            rclpy.logging.get_logger('async_runner').error(f'Coroutine execution failed: {str(e)}')
            raise
    
    def cleanup(self):
        """Clean up the event loop."""
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.thread.join(timeout=1.0)

def with_error_logging(logger_name: str):
    """Decorator to add error logging to async functions."""
    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            logger = rclpy.logging.get_logger(logger_name)
            try:
                return await func(*args, **kwargs)
            except Exception as e:
                logger.error(f'{func.__name__} failed: {str(e)}')
                raise
        return wrapper
    return decorator 