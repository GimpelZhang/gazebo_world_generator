# Copyright (c) 2019 - The Procedural Generation for Gazebo authors
# For information on the respective copyright owner see the NOTICE file
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from ...utils import is_scalar
from . import XMLBase


class XMLInteger(XMLBase):
    _NAME = ''

    def __init__(self, default=0, min_value=None, max_value=None):
        XMLBase.__init__(self)
        assert isinstance(default, float) or isinstance(default, int)
        self._default = default
        self._value = default

        if min_value is not None:
            assert is_scalar(min_value), \
                '[{}] Min. value must be a scalar'.format(
                    self.xml_element_name)
            self._min_value = min_value
        else:
            self._min_value = None

        if max_value is not None:
            if min_value is not None:
                assert max_value > min_value, \
                    '[{}] Max. value {} is not greater than' \
                    ' provided min. value {}'.format(
                        self.xml_element_name, max_value, min_value)
            assert is_scalar(max_value), \
                '[{}] Max. value must be a scalar'.format(
                    self.xml_element_name)
            self._max_value = max_value
        else:
            self._max_value = None

    def _set_value(self, value, min_value=None, max_value=None):
        assert not isinstance(value, bool), \
            '[{}] Input value cannot be a boolean'.format(
                self.xml_element_name)
        assert is_scalar(value), \
            '[{}] Input value must be a float or' \
            ' an integer, received={}'.format(
                self.xml_element_name, value)
        if isinstance(value, float):
            assert value.is_integer(), \
                '[{}] Provided floating point value does' \
                ' not hold an integer value'.format(
                    self.xml_element_name)

        if min_value is not None:
            assert value >= min_value, \
                'Value for {} must be greater or equal to {}'.format(
                    self._NAME, min_value)

        if max_value is not None:
            if min_value is not None:
                assert max_value > min_value, \
                    'Max. value {} for {} is not greater' \
                    ' than provided min. value {}'.format(
                        max_value, self._NAME, min_value)
            assert value <= max_value, \
                'Value for {} must be less or equal to {}'.format(
                    self._NAME, max_value)

        self._value = int(value)

    def reset(self):
        self._value = self._default
        XMLBase.reset(self)

    def is_valid(self):
        if not isinstance(self._value, int):
            print('Integer object must have of type integer')
            return False
        return True

    def get_formatted_value_as_str(self):
        assert self.is_valid(), 'Invalid scalar value'
        return format(self._value, 'd')

    def random(self):
        import random
        self._set_value(random.randint(0, 10))
