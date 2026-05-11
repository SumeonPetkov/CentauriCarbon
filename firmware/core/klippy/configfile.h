#ifndef CONFIGFILE_H
#define CONFIGFILE_H
 
#include <string>
#include <iostream>
#include <string.h>
#include <float.h>
#include <vector>
#include <cstdint>
#include "Define_config_path.h"
// #include "debug.h"
// #include "../partslice_serial_v/printer/printer_para.h"


#define PARA_STATE_OPEN_ERR -1
#define PARA_STATE_PARA_ERR -2
#define PARA_STATE_READ_ERR -3

using std::string;
 
typedef struct Options
{
	string comment;	// save option notes.
	string name;	// save option name.
	string value;	// save option value.
	Options* next;
}*pOption, Option;
 
typedef struct Sections
{
	string comment;	// save section notes.
	string name;	// section name.
	Option *options;
	Sections* next;
}*pSection, Section;

class ConfigParser //INI
{
private:
	string _ini;
	Section *_section_list_head;
	size_t section_count = 0;
 
	string line_break = "\n";
 
public:
	int  para_state;
	ConfigParser(string path);
	ConfigParser(string path, bool force);
	~ConfigParser();
	Section* InitSectionList();
	bool ReadIni();	// read part
	Section* GetSection(const string& root);
	string GetString(const string& root, const string& key, const string& def = "");
	int GetInt(const string& root, const string& key, int def = INT32_MIN, int minval = INT32_MIN, int maxval = INT32_MAX);
	bool GetBool(const string& root, const string& key, bool def = false);
	double GetDouble(const string& root, const string& key, double def = DBL_MIN, double minval = DBL_MIN, double maxval = DBL_MAX, double above = DBL_MIN, double below = DBL_MAX, bool note_valid = true);
	std::vector<double> GetDoubleVector(const string& root, const string& key, double def = DBL_MIN, double minval = DBL_MIN, double maxval = DBL_MAX, double above = DBL_MIN, double below = DBL_MAX, bool note_valid = true);

	void SetValue(const string& root, const string& key, const string& value);	// set ini
	void SetInt(const string& root, const string& key, int value);// void SetString(const string& root, const string& key, const string& value);
	void SetDouble(const string& root, const string& key, double value);
	void SetBool(const string& root, const string& key, bool value);
	void Setuservalue(Section *section);
	bool WriteIni(string path);	// write ini
	bool WriteI_specified_Ini(string path, const string &root, const std::vector<string> &keys);
	bool IsNotes(string line);	// Is notes
	bool IsSection(string line);
	bool IsExistSection(string root);
	bool IsExistOption(string root, string option);
	string TrimLeadTrailSpecStr(string src, char keyword = ' ');
	string TrimLeadSpecStr(string src, string keyword);
	string TrimTrailSpecStr(string src, string keyword);
	string TrimTrialLineBreak(string data);	// delete '\r' & '\n' at the end of the string.
	void AddSection(string section, string note);
	void AddOption(string section, string option, string value, string note);
	void ViewSections();
	std::vector<string> get_prefix_sections(std::string prefix);
	std::vector<string> get_prefix_options(std::string section, std::string prefix);
	std::vector<string> get_all_options(std::string section);
	void DeleteSection(string section);
};

#endif