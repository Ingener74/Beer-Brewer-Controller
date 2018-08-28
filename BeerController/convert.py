#!/usr/bin/env python

import json
from jinja2 import Template, Environment, FileSystemLoader

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
	env = Environment()
	env.loader = FileSystemLoader('.')
	t = env.get_template(template)
	t.globals['counter'] = counter
	t.globals['reset_counter'] = reset_counter
	open(output_file, 'w').write(t.render(json.load(open(data))))

if __name__ == '__main__':
    main('menu.json', 'template_menu.txt', 'Menu.h')
    main('menu.json', 'template_btn.txt', 'BtnHandlers.h')
    main('menu.json', 'template_up.txt', 'UpHandlers.h')
    main('menu.json', 'template_dwn.txt', 'DwnHandlers.h')
