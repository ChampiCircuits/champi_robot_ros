#!/usr/bin/env python3
"""
Match Controller - Orchestrates match execution
Handles timing, scoring, and high-level match logic.
NO ROS dependencies - pure business logic.
"""

from typing import Callable, Optional, Tuple
import time


class MatchController:
    """
    Orchestrates match execution.
    
    Responsibilities:
    - Match timing (start, elapsed, remaining time)
    - Score management
    - Decision to return home based on time
    - Callbacks for external notification (ROS publishing, etc.)
    """
    
    def __init__(self, total_time: float, return_home_safety_margin: float):
        """
        Initialize match controller.
        
        Args:
            total_time: Total match duration in seconds
            return_home_safety_margin: Safety margin for return home calculation in seconds
        """
        self.total_time = total_time
        self.return_home_safety_margin = return_home_safety_margin
        self.start_time: Optional[float] = None
        self.score = 0
        
        # Callbacks for external systems (dependency injection)
        # These will be set by the ROS node to publish updates
        self.on_score_changed: Callable[[int], None] = lambda s: None
        self.on_time_warning: Callable[[float], None] = lambda t: None
        self.on_match_started: Callable[[], None] = lambda: None
        self.on_match_ended: Callable[[], None] = lambda: None
    
    def start_match(self) -> None:
        """Start the match timer."""
        self.start_time = time.time()
        self.score = 0
        self.on_match_started()
    
    def reset(self) -> None:
        """Reset match state."""
        self.start_time = None
        self.score = 0
    
    def is_match_started(self) -> bool:
        """Check if match has started."""
        return self.start_time is not None
    
    def get_elapsed_time(self) -> float:
        """Get elapsed time since match start in seconds."""
        if not self.start_time:
            return 0.0
        return time.time() - self.start_time
    
    def get_remaining_time(self) -> float:
        """Get remaining time until match end in seconds."""
        if not self.start_time:
            return self.total_time
        return self.total_time - self.get_elapsed_time()
    
    def is_match_over(self) -> bool:
        """Check if match time has expired."""
        return self.get_remaining_time() <= 0.0
    
    def should_return_home(self, estimated_time_to_home: float) -> bool:
        """
        Determine if robot should start returning home.
        
        Args:
            estimated_time_to_home: Estimated time to reach home in seconds
            
        Returns:
            True if robot should start returning home
        """
        remaining = self.get_remaining_time()
        return remaining <= (estimated_time_to_home + self.return_home_safety_margin)
    
    def add_points(self, points: int) -> None:
        """
        Add points to the current score.
        
        Args:
            points: Number of points to add
            reason: Optional reason for logging
        """
        self.score += points
        self.on_score_changed(self.score)
    
    def get_score(self) -> int:
        """Get current score."""
        return self.score
    
    def get_match_state(self) -> Tuple[float, float, int]:
        """
        Get complete match state.
        
        Returns:
            Tuple of (elapsed_time, remaining_time, score)
        """
        return (self.get_elapsed_time(), self.get_remaining_time(), self.score)
