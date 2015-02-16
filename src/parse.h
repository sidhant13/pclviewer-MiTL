/*Project: PCL_Visualization
 *Description: Utility class to parse the configuration file which contains user inputs like triangulation options
 *              and color coding for different zzom levels.
 *Methods:
 * parse(@param filename)   [Constructor]
 *                          The constructor for this class, with argument as the relative or absolute path of .ini user input file.
 *                          The default value of parameter is "../config/property.ini"
 *
 * bool getFileReadErrorFlag()  [Method]
 *                              Returns true if constructor for some reasons was not able to open ini file
 *
 * getValue(string key, string& value,string default_value)  [Method]
 *                              Finds the value related to the key in .ini file and sets the 'value' of the 'key' to the
 *                              one provided by 'default_value' if the key is not found.
 *
 * int nameToRgb(std::string name)      [Method]
 *                                      Utility function that returns the rgb value of the color represented by the name parameter.
*/


#include <iostream>
#include <fstream>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/exceptions.hpp>
#include <exception>

#ifndef PARSE_H
#define PARSE_H

class parse
{
public:
    parse(std::string filename="../config/property.ini");
    void getValue(const std::string key, std::string& value, std::string default_value);
    bool getFileReadErrorFlag();
    int nameToRgb(std::string name);

private:
    boost::property_tree::ptree pt;
    bool file_read_error_flag;
    int rgb;

};

#endif // PARSE_H
