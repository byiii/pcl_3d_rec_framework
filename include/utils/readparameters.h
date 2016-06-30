#ifndef READPARAMETERS_H
#define READPARAMETERS_H

#include <iostream>
#include <string>
#include <libconfig.h++>

class readParameters
{
    std::string configure_file_;
    libconfig::Config cfg_;
public:
    readParameters()
        :configure_file_("")
    {}

    template<typename Scalar>
    void get(const char* key, Scalar &value);

    //------------------------------------------------------------
    void setConfigureFile(const char* file)
    {
        configure_file_ = file;
        using namespace std;

        // Read the file. If there is an error, report it and exit.
        try
        {
            cfg_.readFile(configure_file_.c_str());
        }
        catch(const libconfig::FileIOException &fioex)
        {
            cerr << "I/O error while reading file." << endl;
            cerr << fioex.what() << endl;
        }
        catch(const libconfig::ParseException &pex)
        {
            cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
                 << " - " << pex.getError() << endl;
        }
    }
};

template<typename Scalar>
void readParameters::get(const char *key, Scalar &value)
{
    using namespace std;

    try
    {
        const libconfig::Setting& root = cfg_.getRoot();
        if(root.exists(key))
            root.lookupValue(key, value);
    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        cerr << "some settings not found, default value will be used." << endl;
        cerr << nfex.what() << endl;
    }
}

#endif // READPARAMETERS_H
