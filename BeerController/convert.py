#!/usr/bin/env python

import json
from jinja2 import Template

id_counter = 0

def counter():
	global id_counter
	id_counter += 1
	return '{}'.format(id_counter)

def reset_counter():
	global id_counter
	id_counter = 0
	return ''

def main(data, template, output_file):
	t = Template(open(template).read())
	t.globals['counter'] = counter
	t.globals['reset_counter'] = reset_counter
	open(output_file, 'w').write(t.render(json.load(open(data))))


if __name__ == '__main__':
    main('menu.json', 'template.txt', 'Menu.h')