#!/usr/bin/env python

from __future__ import print_function
import rospy
import unittest
import subprocess
import os

PKG = 'quaro_description'
NAME = 'test_urdf_files'

import roslib
roslib.load_manifest(PKG)


def call_xacro(xml_file):
    assert os.path.isfile(xml_file), 'Invalid XML xacro file'
    return subprocess.check_output(['xacro', '--inorder', xml_file])


class TestquaroURDFFiles(unittest.TestCase):
    def test_xacro(self):
        # Retrieve the root folder for the tests
        test_dir = os.path.abspath(os.path.dirname(__file__))
        robots_dir = os.path.join(test_dir, '..', 'robots')

        for item in os.listdir(robots_dir):
            if 'quaro-foo' in item:
                continue
            if not os.path.isfile(os.path.join(robots_dir, item)):
                continue
            output = call_xacro(os.path.join(robots_dir, item))
            self.assertNotIn(
                output,
                'XML parsing error',
                'Parsing error found for file {}'.format(item))
            self.assertNotIn(
                output,
                'No such file or directory',
                'Some file not found in {}'.format(item))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, NAME, TestRexROVURDFFiles)
