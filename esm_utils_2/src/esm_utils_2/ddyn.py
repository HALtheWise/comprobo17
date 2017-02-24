#!/usr/bin/env python

"""
Dynamic dynamic reconfigure server.

Just register your variables for the dynamic reconfigure
and call start with a callback.

Author: Sammy Pfeiffer
"""

from dynamic_reconfigure.server import Server
from dynamic_reconfigure.parameter_generator_catkin import ParameterGenerator
from dynamic_reconfigure.encoding import extract_params
from rospkg import RosPack
import rospy


class DDynamicReconfigure(ParameterGenerator):
    """Dynamic reconfigure server that can be instanced directly."""

    def __init__(self, name=None):
        global id
        self.group = self.Group(self, "Default", "", True, 0, 0)
        id = 1
        if name is None:
            self.name = rospy.get_name() + "_dyn_rec"
        else:
            self.name = name
        self.constants = []
        rp = RosPack()
        self.dynconfpath = rp.get_path('dynamic_reconfigure')

        self.dyn_rec_srv = None

    def get_type(self):
        class TypeClass(object):
            def __init__(self, config_description):
                self.config_description = config_description
                self.min = {}
                self.max = {}
                self.defaults = {}
                self.level = {}
                self.type = {}
                self.all_level = 0

                for param in extract_params(config_description):
                    self.min[param['name']] = param['min']
                    self.max[param['name']] = param['max']
                    self.defaults[param['name']] = param['default']
                    self.level[param['name']] = param['level']
                    self.type[param['name']] = param['type']
                    self.all_level = self.all_level | param['level']

        return TypeClass(self.group.to_dict())

    def add_variable(self, name, description, default=None, min=None, max=None, edit_method=""):
        """Register variable, like gen.add() but deducting the type"""
        if type(default) == int:
            if edit_method == "":
                self.add(name, "int", 0, description, default, min, max)
            else:  # enum
                self.add(name, "int", 0, description, default, min, max, edit_method)
        elif type(default) == float:
            self.add(name, "double", 0, description, default, min, max)
        elif type(default) == str:
            self.add(name, "str", 0, description, default)
        elif type(default) == bool:
            self.add(name, "bool", 0, description, default)

        return default

    def get_variable_names(self):
        """Return the names of the dynamic reconfigure variables"""
        names = []
        for param in self.group.parameters:
            names.append(param['name'])
        return names

    def start(self, callback):
        self.dyn_rec_srv = Server(self.get_type(), callback, namespace=self.name)


def NewParam(default, min=None, max=None, name=None, description=None, edit_method=""):
    return {'request': 'New',
            'default': default,
            'min': min,
            'max': max,
            'name': name,
            'description': description,
            'edit_method': edit_method}

class Params(object):
    def __init__(self, name=None):
        self._ddr = DDynamicReconfigure(name)
        self.names = dict()

    @staticmethod
    def _infer_type(default, edit_method):
        if type(default) == int:
            if edit_method == '':
                return "int"
            else:  # enum
                return "enum"
        elif type(default) == float:
            return "float"
        elif type(default) == str:
            return "str"
        elif type(default) == bool:
            return "bool"

    def __infer_max(self, default):
        bases = [1, 2, 5]
        margin = 2.0

        exp = 0
        while True:
            for base in bases:
                max = base * 10 ** exp
                if max >= margin * default:
                    return max
            exp += 1

    def __add(self, name, default, description, min, max, edit_method):
        type = self._infer_type(default, edit_method)

        if description is None:
            description = "{name} ({type})".format(name=name, type=type)

        if type in ('int', 'float'):
            # Attempt to infer min and max
            if max == None:
                max = self.__infer_max(default)

            if min == None:
                if default < 0:
                    min = -max
                else:
                    min = 0
        print type
        self._ddr.add_variable(name, description, default, min, max, edit_method)

    def __setattr__(self, key, value):
        if 'names' in self.__dict__ and key in self.names:
            # We already have this parameter, set it.
            print 'Setting already-existing parameter {}'.format(key)
            return

        if type(value) in [int, float, str]:
            value = NewParam(value)

        if type(value) == dict:
            if 'request' in value and value['request'] == 'New':
                del value['request']
                # We are being asked to make a new parameter
                if 'name' not in value or value['name'] is None:
                    value['name'] = key

                self.names[key] = value['name']

                self.__add(**value)
                print 'Parameter added: {}'.format(value['name'])

                return


        print "Unexpected key-value call {} {}".format(key, value)
        super(Params, self).__setattr__(key, value)
        return

    def New(self, *arg, **args):
        return NewParam(*arg, **args)


params = Params()
