#!/usr/bin/env python3

import hebi
from time import sleep

lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate
sleep(2.0)

family_name = "Test Family"
module_names = ["J3_elbow", "J2_shoulder", "J1_base"]

group = lookup.get_group_from_names([family_name], module_names)

if group is None:
    print('Group not found: Did you forget to set the module family and name above?')
    exit(1)

print('Found group on network with {0} modules.'.format(group.size))
