#!/usr/bin/env python3
"""
Utilities for handling table symmetry between yellow and blue teams.

The game table has vertical axis symmetry (x=1.5m center for 3m wide table).
Yellow team starts on the left, blue team starts on the right.
Each element has a 'mirror_id' pointing to its symmetric counterpart.

For detailed documentation about the symmetry system, see:
    champi_brain/docs/table_symmetry.md
"""

from typing import Dict, Any
from champi_brain.enums import Color


class SymmetryMapper:
    """Handles conversion between yellow and blue team element IDs using mirror_id mapping."""
    
    def __init__(self, elements_config: list[Dict[str, Any]], zones_config: list[Dict[str, Any]]):
        """
        Initialize the symmetry mapper with element and zone configurations.
        
        Args:
            elements_config: List of element dicts from YAML (must include 'id' and 'mirror_id')
            zones_config: List of zone dicts from YAML (must include 'id' and 'mirror_id')
        """
        # Build bidirectional mapping for elements
        self.element_mirror_map: Dict[str, str] = {}
        for elem in elements_config:
            elem_id = elem['id']
            mirror_id = elem.get('mirror_id', elem_id)  # Default to self if no mirror_id
            self.element_mirror_map[elem_id] = mirror_id
        
        # Build bidirectional mapping for zones
        self.zone_mirror_map: Dict[str, str] = {}
        for zone in zones_config:
            zone_id = zone['id']
            mirror_id = zone.get('mirror_id', zone_id)  # Default to self if no mirror_id
            self.zone_mirror_map[zone_id] = mirror_id
    
    def get_element_id_for_color(self, element_id: str, team_color: Color) -> str:
        """
        Get the appropriate element ID for the given team color.
        
        Args:
            element_id: The base element ID (typically for yellow team)
            team_color: The team color (YELLOW or BLUE)
        
        Returns:
            The element ID to use: original ID for yellow, mirror_id for blue
        """
        if team_color == Color.BLUE:
            return self.element_mirror_map.get(element_id, element_id)
        return element_id
    
    def get_zone_id_for_color(self, zone_id: str, team_color: Color) -> str:
        """
        Get the appropriate zone ID for the given team color.
        
        Args:
            zone_id: The base zone ID (typically for yellow team)
            team_color: The team color (YELLOW or BLUE)
        
        Returns:
            The zone ID to use: original ID for yellow, mirror_id for blue
        """
        if team_color == Color.BLUE:
            return self.zone_mirror_map.get(zone_id, zone_id)
        return zone_id
    
    def get_all_element_ids_for_color(self, team_color: Color) -> list[str]:
        """
        Get all element IDs that should be used for the given team color.
        
        Args:
            team_color: The team color (YELLOW or BLUE)
        
        Returns:
            List of element IDs appropriate for this team
        """
        if team_color == Color.YELLOW:
            # For yellow, use all IDs that are on the left side
            # This means IDs that are NOT mirror_ids of others (or self-symmetric)
            yellow_ids = []
            for elem_id, mirror_id in self.element_mirror_map.items():
                # Include if it's the "primary" one (comes before its mirror alphabetically)
                # or if it's self-symmetric
                if elem_id == mirror_id or elem_id < mirror_id:
                    yellow_ids.append(elem_id)
            return yellow_ids
        else:
            # For blue, use the mirror of all yellow elements
            yellow_ids = self.get_all_element_ids_for_color(Color.YELLOW)
            return [self.element_mirror_map.get(eid, eid) for eid in yellow_ids]


# Global instance to be initialized by the node
_symmetry_mapper: SymmetryMapper | None = None


def init_symmetry_mapper(elements_config: list[Dict[str, Any]], zones_config: list[Dict[str, Any]]) -> None:
    """Initialize the global symmetry mapper instance."""
    global _symmetry_mapper
    _symmetry_mapper = SymmetryMapper(elements_config, zones_config)


def get_element_id_for_color(element_id: str, team_color: Color) -> str:
    """
    Convenience function to get element ID for color using global mapper.
    
    Args:
        element_id: The base element ID
        team_color: The team color
    
    Returns:
        The appropriate element ID for this team
    
    Raises:
        RuntimeError: If symmetry mapper hasn't been initialized
    """
    if _symmetry_mapper is None:
        raise RuntimeError("Symmetry mapper not initialized. Call init_symmetry_mapper first.")
    return _symmetry_mapper.get_element_id_for_color(element_id, team_color)


def get_zone_id_for_color(zone_id: str, team_color: Color) -> str:
    """
    Convenience function to get zone ID for color using global mapper.
    
    Args:
        zone_id: The base zone ID
        team_color: The team color
    
    Returns:
        The appropriate zone ID for this team
    
    Raises:
        RuntimeError: If symmetry mapper hasn't been initialized
    """
    if _symmetry_mapper is None:
        raise RuntimeError("Symmetry mapper not initialized. Call init_symmetry_mapper first.")
    return _symmetry_mapper.get_zone_id_for_color(zone_id, team_color)
