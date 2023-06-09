import pyglet


class SimObject(pyglet.sprite.Sprite):
    """
    A class to define a generic object in the simulation.
    Most objects in the simulation, i.e. robot, obstacles, lights are derived from this class.
    This class itself is derived from the pyglet.sprite.Sprite class
    """

    def __init__(self, *args, **kwargs):
        """
        Initializes the class object.
        :param args: additional positional arguments
        :param kwargs: additional keyword arguments
        """
        super(SimObject, self).__init__(*args, **kwargs)

        # Declaring all the member variables of the class
        self.type = None            # Specifies the type of the object - robot, obstacles, lights etc.
        self.child_objects = []     # List of objects that can be spawned by this object
        self.sim_state = None       # Simulation state object
        self.dead = False           # whether this object has to be removed from screen or not
        self.collision_radius = 0   # circle collider radius
        self.collider_type = None   # "circle" or "polygon"

    def update_object(self, dt):
        """
        Virtual update_object function. This is not named update because the Sprite object has a
        function called update
        """
        pass


    def handle_collision_with(self, other_object):
        self.dead = False