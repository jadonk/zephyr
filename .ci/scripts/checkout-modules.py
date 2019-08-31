#!/usr/bin/env python3

import os
import subprocess
import sys
import yaml

class Dummy(object):
	def __init__(self):
		pass

def runcmd( cmd ):
	print('running command: "{}"'.format(cmd))
	r = subprocess.run( cmd.split() )
	if 0 != r.returncode:
		print('command failed: {}'.format(cmd))
		print('return code: {}'.format(r.returncode))
		print('stdout:\n{}'.format(r.stdout))
		print('stderr:\n{}'.format(r.stderr))
		raise IOError('{} => {}'.format(cmd, r.stderr))

def main( argv ):
	"""
	Usage: checkout-modules [path to west.yml] [path to zephyrproject dir] [path to ZEPHYR_MODULES.txt]

	This script will parse the file named in the first argument (west.yml) and checkout modules from git to the path
	named in the second argument (zephyrproject directory).

	It checks each resource fetched in git to see if it has as zephyr/module.yml entry (or equivalently a zephyr/CMakeLists.txt
	and Kconfig). If so, the (absolute) module path is exported to the file named in the third argument (ZEPHYR_MODULES.txt).

	For more information, see "Building Zephyr without West"
	https://docs.zephyrproject.org/latest/guides/west/without-west.html#no-west
	"""

	westYamlPath = os.path.abspath(argv[1])
	print('using westYamlPath="{}"'.format(westYamlPath))
	zephyrProjectPath = os.path.abspath(argv[2])
	print('using zephyrProjectPath="{}"'.format(zephyrProjectPath))
	zephyrModulesPath = os.path.abspath(argv[3])
	print('using zephyrModulesPath="{}"'.format(zephyrModulesPath))

	with open(westYamlPath) as westYaml:
		data = yaml.safe_load(westYaml)
		defaults = data['manifest']['defaults']
		remotes = data['manifest']['remotes']
		projects = data['manifest']['projects']

		if not isinstance(defaults, dict):
			raise ValueError('expected defaults to be of type dict not of type {}'.format(type(defaults)))

		if not isinstance(remotes, list):
			raise ValueError('expected remotes to be of type list not of type {}'.format(type(remotes)))

		if not isinstance(projects, list):
			raise ValueError('expected projects to be of type list not of type {}'.format(type(projects)))

		xdefaults = Dummy()
		for k in defaults:
			v = defaults[ k ]
			setattr(xdefaults,k.replace('-','_'),v)

		xremotes = {}
		for remote in remotes:
			name = remote['name']
			xremotes[name] = Dummy()
			for k in remote:
				v = remote[k]
				setattr(xremotes[name],k.replace('-','_'),v)

		xprojects = {}
		for project in projects:
			name = project['name']
			xprojects[name] = Dummy()
			for k in project:
				v = project[k]
				setattr(xprojects[name],k.replace('-','_'),v)

		zephyrProjectDir = os.path.abspath(zephyrProjectPath)
		zephyrModules = ''
		for name in xprojects:
			project = xprojects[ name ]
			path = zephyrProjectDir + '/../' + project.path
			try:
				remote = project.remote
			except:
				remote = xdefaults.remote

			url = xremotes[remote].url_base + '/' + project.name
			print('checking out {} to {}'.format(url, path))

			os.makedirs(path, mode=0o777, exist_ok=True)
			os.chdir(path)

			# sadly, there is no way to check out a single commit from GitHub :(
			# https://twitter.com/cfriedt/status/1167232797070770176
			runcmd('git clone {} .'.format(url))
			runcmd('git checkout -b detached {}'.format(project.revision))

			if os.path.exists('zephyr/module.yml') or (os.path.exists('zephyr/CMakeLists.txt') and os.path.exists('Kconfig')):
				zephyrModules += path + ';'
			else:
				print('not adding {} to modules, because no module definition exists'.format(project.name))

			os.chdir(zephyrProjectDir)

		with open(zephyrModulesPath,'w') as zephyrModulesFile:
			zephyrModulesFile.write(zephyrModules)
			zephyrModulesFile.close()

if __name__ == '__main__':
	sys.exit( main( sys.argv ) )
