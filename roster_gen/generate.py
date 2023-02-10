import os

if __name__ == "__main__":
	OUT_PATH = './roster.txt'

	if (os.path.isfile(OUT_PATH)):
		outFile = open(OUT_PATH, 'r+')
	else:
		outFile = open(OUT_PATH, 'x')

	definedLines = outFile.readlines()
	def isDefined(text):
		for line in definedLines:
			if text in line:
				return True
		return False

	foundFiles = []

	rosTypes = {
		'subscription': 'topic', 
		'service': 'service',
		'client': 'service',
		'publisher': 'topic'
	}
	updates = 0
	fileAdditions = 0
	def appendLine(path, msgType, rosType, file, comment):
		global fileAdditions
		defined = isDefined(path)
		if not(defined):
			try:
				line = msgType + " " + rosTypes[rosType] + ' ' + path + ', a ' + rosType + ' in ' + file + ' ' + comment + '\n\n'
				outFile.write(line)
				print('Added ' + path)
				fileAdditions = fileAdditions + 1
			except:
				print('Object did not load correctly')

	def fixPath(path):
		if path == '' or path == None:
			return "'ERROR'"
		if ' ' in path:
			path = path[0:path.find(' ')]
		if path[0] == '"':
				path = "'" + path[1:-1] + "'"
		return path

	def searchFile(path):
		file = open(path, 'r')
		lines = file.readlines()

		for line in lines:
			if path[-3:] == '.py':
				if 'self.create_' in line:
					cut = line[line.find('self.create_') + len('self.create_'):-1]
					argStartPos = cut.find('(')
					rosType = cut[0:argStartPos]
					if not(rosType == 'timer'):
						args = cut[argStartPos + 1:cut.find(')')].split(', ')
						if '#' in cut:
							comment = '    ' + cut[cut.find('#'):]
						else:
							comment = ''
						try:
							appendLine(fixPath(args[1]), args[0], rosType, path[1:], comment)
						except:
							print('Object did not load correctly')
			elif path[-4:] == '.cpp':
				if '->create_' in line:
					cut = line[line.find('->create_') + len('->create_'):-1]
					argStartPos = cut.find('(')
					rosType = cut[0:cut.find('<')]
					args = cut[argStartPos + 1:cut.find(')')].split(', ')
					if '//' in cut:
						comment = '    ' + cut[cut.find('//'):]
					else:
						comment = ''
					msgType = cut[cut.find('<'):cut.find('>')]
					msgType = msgType[msgType.rfind(':') + 1:]
					try:
						appendLine(fixPath(args[0]), msgType, rosType, path[1:], comment)
					except:
						print('Object did not load correctly')
		file.close()

	def searchDir(path):
		files = os.listdir(path[0:len(path) - 1])
		for item in files:
			if os.path.isdir(path + item) and item != 'build':
				searchDir(path + item + '/')
			elif (item[-3:] == '.py' or item[-4:] == '.cpp') and path != './':
				foundFiles.append(path + item)
				print('Found: ' + path + item)

	def generate():
		print('Searching for files..')
		cwd = os.listdir('./')
		# Switch between searching all folders or just src
		if 'src' in cwd:
			searchDir('./src')
		else:
			searchDir('./')

		print('Parsing Files...')
		for file in foundFiles:
			searchFile(file)

		print('Done!')
		print('Added ' + str(fileAdditions) + ' lines')
		outFile.close()

	generate()
