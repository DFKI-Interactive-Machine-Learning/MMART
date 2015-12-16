/*
 * This file is part of the AV-NUI project.
 * Copyright (C) 2013 DFKI GmbH. All rights reserved.
 *
 * Disclaimer:
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND
 * CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#include "core/NUIDLLexport.hpp"
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/function.hpp>
#include <set>
#include <map>

class NUIconfig;

namespace nui {
CONFIG_EXPORT NUIconfig* config();
CONFIG_EXPORT void configname(std::string);
}

/**
 * \brief NUI global configuration.
 *
 */
class NUIconfig {
public:
	/**
	 * \brief Default constructor.
	 */
	NUIconfig();
	/**
	 * \brief Default destructor.
	 */
	~NUIconfig() {}

	/**
	 * \brief Returns bool value for config.
	 * \param s - config path
	 * \param b - default value
	 * \return config value
	 */
	CONFIG_EXPORT
	bool get(const std::string s, const bool b);

	/**
	 * \brief Returns bool value for config.
	 * \param s - config path
	 * \param b - default value
	 * \return config value
	 */
	CONFIG_EXPORT
	float get(const std::string s, const float f);


	/**
	 * \brief Returns int value for config.
	 * \param s - config path
	 * \param i - default value
	 * \return config value
	 */
	CONFIG_EXPORT
	int get(const std::string s, const int i);

	/**
	 * \brief Returns string value for config.
	 * \param s - config path
	 * \param str - default value
	 * \return config value
	 */
	CONFIG_EXPORT
	std::string getString(const std::string s, const std::string str);

	/**
	 * \brief Changes bool value for config.
	 * \param s - config path
	 * \param b - new value
	 */
	CONFIG_EXPORT
	void changeValue(const std::string s, const bool b);

	/**
	 * \brief Changes float value for config.
	 * \param s - config path
	 * \param f - new value
	 */
	CONFIG_EXPORT
	void changeValue(const std::string s, const float f);

	/**
	 * \brief Changes int value for config.
	 * \param s - config path
	 * \param i - new value
	 */
	CONFIG_EXPORT
	void changeValue(const std::string s, const int i);

	/**
	 * \brief Changes string.
	 * \param s - config path
	 * \param str - new value
	 */
	CONFIG_EXPORT
	void changeString(const std::string s, const std::string str);

	/**
	 * \brief Returns singleton instance of config.
	 * \return instance
	 */
	CONFIG_EXPORT
	static NUIconfig* instance();

	/**
	 * \brief Sets the name of the config.
	 * \param name
	 */
	CONFIG_EXPORT
	static void setConfigName(std::string );

	CONFIG_EXPORT
	boost::filesystem::path getBasePath();

	CONFIG_EXPORT
	std::string getConfigFilePathString();

	CONFIG_EXPORT
	boost::property_tree::ptree subTree(std::string path);

	CONFIG_EXPORT
	void applyConfigChanges();

	CONFIG_EXPORT
	void reloadConfigFile();

	CONFIG_EXPORT
	void registerForReconfigure(const std::string configPath,
			boost::function<void()> reconfigureFunction);

private:
	/**
	 * \brief Write config file.
	 */
	void writeConfigFile();
	void checkType(const std::string s, const std::string type);
	void storeChangedPath(const std::string path);
	static NUIconfig* s_instance;
	static std::string s_config_name;
	boost::property_tree::ptree _baseTree;
	boost::property_tree::ptree _nuiTree;
	boost::property_tree::ptree _nuiTreeTemp;
	boost::filesystem::path _basePath;
	std::set<std::string> _reconfPaths;
	std::multimap<std::string, boost::function<void()> > _registeredPaths;
	std::string _configFilePath;

};
