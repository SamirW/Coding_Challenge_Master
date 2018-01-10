import yaml
from region import Obstacle

class Environment:
    def __init__(self, bounds=(0,100,0,100), yaml_file=None):
        self._bounds = bounds
        self._obstacles = []
        if not yaml_file is None:
            self.load_from_yaml_file(yaml_file)

    @property
    def bounds(self):
        return self._bounds

    @property 
    def obstacles(self):
        return self._obstacles

    def add_obstacles(self, obstacles):
        self.obstacles = self.obstacles + obstacles

    def load_from_yaml_file(self, yaml_file):
        f = open(yaml_file)
        self.data = yaml.safe_load(f)
        f.close()
        self.parse_yaml_data(self.data)

    def parse_yaml_data(self, data):
        if 'environment' in data:
            env = data['environment']
            self.parse_yaml_obstacles(env['obstacles'])
        else:
            raise Exception("environment not found in YAML file")

    def parse_yaml_obstacles(self, obstacles):
        for name, data in obstacles.iteritems():
            obs = Obstacle()
            for point in data['points']:
                obs.add_point(tuple(point))
            self.add_obstacles([obs])