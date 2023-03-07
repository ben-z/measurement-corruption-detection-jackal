#!/usr/bin/env python3

# This script checks if a given string is present in a comma-separated list of strings
# passed as a command-line argument. If the string is found, the script prints a success message.
# If the string is not found, the script raises a ValueError with an error message.

# Example usage:
#   $ python script.py hello "world,hello,python"
#   Output: hello found in the list of strings

#   $ python script.py foo "bar,baz,qux"
#   Output: ValueError: foo not found in the list of strings


import sys

if len(sys.argv) < 3:
    print(f"Usage: {sys.argv[0]} <string> <comma-separated-list-of-strings>")
    sys.exit(1)

search_string = sys.argv[1]
list_of_strings = sys.argv[2].split(",")

if search_string not in list_of_strings:
    raise ValueError(f"{search_string} not found in {list_of_strings}")

print(f"{search_string} found in {list_of_strings}")
