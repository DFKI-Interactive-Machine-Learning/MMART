#include "config/NUIconfig.hpp"
#include "core/common.hpp"
#include <boost/property_tree/xml_parser.hpp>

#ifdef WIN32 
#define HOME "ProgramData" ///\todo check that it should not be %APPDATA%  instead of %HOMEPATH% 
#else 
#define HOME "HOME" 
#endif 

NUIconfig* NUIconfig::s_instance = 0;
std::string NUIconfig::s_config_name = "NUI";

NUIconfig* nui::config() {
	return NUIconfig::instance();
}

void nui::configname(std::string name) {
	NUIconfig::setConfigName(name);
}

void NUIconfig::setConfigName(std::string name)
{
	NUIconfig::s_config_name = name;
}

NUIconfig* NUIconfig::instance() {
	if (!s_instance)
		s_instance = new NUIconfig();
	return s_instance;
}

NUIconfig::NUIconfig() {
	s_instance = this;
	// First check if there is a config file in the runtime directory
	_configFilePath = (bfs::current_path() / configFilename).string();
	if (boost::filesystem::exists(_configFilePath))
	{
		reloadConfigFile();
		return;
	} 
	else
	{
		BOOST_LOG_TRIVIAL(warning) << "No configuration file in working directory. [dir:=" << _configFilePath << "]";
	}
	// If there is no local file, check the home path
	char* home_dir = getenv(HOME);
	boost::filesystem::path applicationPath(home_dir);
	applicationPath /= NUIconfig::s_config_name;
	_basePath = applicationPath;
	boost::filesystem::path conf_path = _basePath / "conf";
	if (!boost::filesystem::exists(conf_path))
	{
		boost::filesystem::create_directories(conf_path);
	}
	_configFilePath = (conf_path / configFilename).string();
	reloadConfigFile();
}

bool NUIconfig::get(const std::string s, const bool b) {
	int result = b;
	try {
		return _nuiTree.get<bool>(s);
	} catch (std::exception& e) {
		BOOST_LOG_TRIVIAL(error)<< e.what();
		_nuiTree.put(s, b);
		writeConfigFile();
	}
	checkType(s, "bool");
	return result;
}

float NUIconfig::get(const std::string s, const float f) {
	int result = f;
	try {
		return _nuiTree.get<float>(s);
	} catch (std::exception& e) {
		BOOST_LOG_TRIVIAL(error)<< e.what();
		_nuiTree.put(s, f);
		writeConfigFile();
	}
	checkType(s, "float");
	return result;
}

int NUIconfig::get(const std::string s, const int i) {
	int result = i;
	try {
		result = _nuiTree.get<int>(s);
	} catch (std::exception& e) {
		BOOST_LOG_TRIVIAL(error)<< "Error retrieving config option " << s << ": " << e.what();
		_nuiTree.put(s, i);
		writeConfigFile();
	}
	checkType(s, "int");
	return i;
}

std::string NUIconfig::getString(const std::string s, const std::string str) {
	std::string result = str;
	try {
		result = _nuiTree.get<std::string>(s);
	} catch (std::exception& e) {
		BOOST_LOG_TRIVIAL(error)<< e.what();
		_nuiTree.put(s, str);
		writeConfigFile();
	}
	checkType(s, "string");
	return result;
}

void NUIconfig::checkType(const std::string s, const std::string type) {
	try {
		std::string attrType = _nuiTree.get<std::string>(s + ".<xmlattr>.type");
		if (attrType != type)
			BOOST_LOG_TRIVIAL(warning) << "Config-Option " << s << " - Type mismatch: " << type << " expected, but config type is " << attrType;
		} catch (std::exception& e) {
			BOOST_LOG_TRIVIAL(error) << "Error retrieving type info for " << s << ": " << e.what();
			_nuiTree.put(s + ".<xmlattr>.type", type);
			writeConfigFile();
		}
	}

void NUIconfig::registerForReconfigure(const std::string configPath,
		boost::function<void()> reconfigureFunction) {
	std::pair<std::string, boost::function<void()> > reg (configPath,
						reconfigureFunction);
	_registeredPaths.insert(reg);
}

void NUIconfig::changeValue(const std::string s, const bool b) {
	storeChangedPath(s);
	_nuiTreeTemp.put(s, b);
}

void NUIconfig::changeValue(const std::string s, const float f) {
	storeChangedPath(s);
	_nuiTreeTemp.put(s, f);
}

void NUIconfig::changeValue(const std::string s, const int i) {
	storeChangedPath(s);
	_nuiTreeTemp.put(s, i);
}

void NUIconfig::changeString(const std::string s, const std::string str) {
	storeChangedPath(s);
	_nuiTreeTemp.put(s, str);
}

void NUIconfig::storeChangedPath(const std::string path) {
	char * str = new char[path.size() + 1];
	char * token;
	strcpy(str, path.c_str());
	token = strtok(str, ".");
	std::string tokenConCat = token;
	std::string placeholderToken = "";
	while (token != NULL) {
		_reconfPaths.insert(tokenConCat);
		_reconfPaths.insert(
				"*"
						+ path.substr(tokenConCat.length(),
								path.length() - tokenConCat.length()));
		token = strtok(NULL, ".");
		if (token)
			tokenConCat = tokenConCat + "." + token;
	}
}

void NUIconfig::reloadConfigFile() {
	_baseTree.clear();
	_nuiTree.clear();
	_nuiTreeTemp.clear();
	_reconfPaths.clear();
	try {
		BOOST_LOG_TRIVIAL(info)<< "Reading config file " << _configFilePath;
		read_xml(_configFilePath, _baseTree, boost::property_tree::xml_parser::trim_whitespace);
	} catch (std::exception& e) {
		BOOST_LOG_TRIVIAL(error) << "Error reading config file: " << e.what();
		writeConfigFile();
	}

	try {
		_nuiTree = _baseTree.get_child("NUI_Config");
		_nuiTreeTemp = _nuiTree;
	} catch (std::exception& e) {
		BOOST_LOG_TRIVIAL(error)<< "Error getting NUI_Config subtree: " << e.what();
	}
	writeConfigFile();
}

void NUIconfig::applyConfigChanges() {
	BOOST_LOG_TRIVIAL(info)<< "NUIconfig::applyConfigChanges()";
	std::string output = "Paths to reconfigure:   ";

	for (std::set<std::string>::iterator it=_reconfPaths.begin(); it!=_reconfPaths.end(); ++it) {
		output = output + " " + *it+ "\n";
	}
	BOOST_LOG_TRIVIAL(info) << output;

	_nuiTree = _nuiTreeTemp;
	writeConfigFile();

	output = "Registered Paths:   ";
	for (std::multimap<std::string,boost::function<void ()> >::iterator it=_registeredPaths.begin(); it!=_registeredPaths.end(); ++it) {
		output = output + " " + (*it).first + " => " + "(function)" + "\n";
		if(_reconfPaths.count((*it).first) > 0) {
			(*it).second();
		}
	}
	BOOST_LOG_TRIVIAL(info) << output;

	_reconfPaths.clear();
}

void NUIconfig::writeConfigFile() {
	BOOST_LOG_TRIVIAL(info)<< "Writing config file" << _configFilePath;
	_baseTree.put_child("NUI_Config", _nuiTree);
	_nuiTreeTemp.clear();
	_nuiTreeTemp = _nuiTree;
	//boost::property_tree::xml_writer_settings<char> settings('\t', 1);
	//TODO CHECK : write_xml(_configFilePath, _baseTree, std::locale(), settings);
}

boost::filesystem::path NUIconfig::getBasePath() {
	return _basePath;
}

std::string NUIconfig::getConfigFilePathString() {
	return _configFilePath;
}

boost::property_tree::ptree NUIconfig::subTree(std::string path) {
	boost::property_tree::ptree result;
	try {
		result = _nuiTree.get_child(path);
	} catch (std::exception& e) {
		BOOST_LOG_TRIVIAL(error)<< "Error getting NUIconfig subtree " << path << ": " << e.what();
	}
	return result;
}
