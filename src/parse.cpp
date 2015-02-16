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


#include "parse.h"


parse::parse(std::string filename)
{
    try{
        std::cout << "[Parse] Reading configuration file " << filename << std::endl;
        boost::property_tree::ini_parser::read_ini(filename, pt);
    }
    catch(boost::property_tree::ini_parser::ini_parser_error e){
        std::cout << "[Parse] Could not open " <<  filename << " : "  << e.message() << std::endl;
        file_read_error_flag=true;
    }
    file_read_error_flag=false;
}

void parse::getValue(const std::string key, std::string& value, std::string default_value)
{
    std::string cKey;
    cKey="Main." + key;
    value= pt.get<std::string>(cKey,default_value);
    std::cout << "[Parse] Key "<< key << " mapped to value " << value << std::endl;
}

bool parse::getFileReadErrorFlag(){
    return file_read_error_flag;
}


int parse::nameToRgb(std::string name)
{
    if(name=="Red")
        rgb=255 << 16 | 0 << 8 | 0;
    else if(name=="Blue")
        rgb=0 << 16 | 255 << 8 | 0;
    else if(name=="Lime")
        rgb=0 << 16 | 0 << 8 | 255;
    else if(name=="Orange")
        rgb=255 << 16 | 165 << 8 | 0;
    else if(name=="Yellow")
        rgb=255 << 16 | 255 << 8 | 0;
    else if(name=="Magenta")
        rgb=255 << 16 | 0 << 8 | 255;
    else if(name=="Olive")
        rgb=128 << 16 | 128 << 8 | 0;
    else if(name=="Green")
        rgb=0 << 16 | 128 << 8 | 0;
    else if(name=="Purple")
        rgb=128 << 16 | 128 << 8 | 0;

    else{
        name="Grey";
        rgb=127 << 16 | 127 << 8 | 127;
    }
    std::cout << "[parse] Point Cloud Color set to " << name << std::endl;
    return rgb;
}
