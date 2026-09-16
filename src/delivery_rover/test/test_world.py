# FILE: src/delivery_rover/test/test_world.py
from delivery_rover import world


def test_home_is_free():
    assert not world.is_occupied(*world.LOCATIONS["home"][:2])


def test_shelf_two_is_occupied():
    assert world.is_occupied(6.3, 2.0)


def test_outside_counts_as_occupied():
    assert world.is_occupied(-1.0, 1.0)


def test_grid_matches_resolution():
    g = world.occupancy_grid()
    assert g.shape == (
        int(world.SIZE_Y / world.RESOLUTION),
        int(world.SIZE_X / world.RESOLUTION),
    )
