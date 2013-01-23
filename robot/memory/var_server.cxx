#include "stdheader.h"

template <class T>
class VarServer{
private:
    map<string,T> vars;
public:
    VarServer(string filename){
        load_vars(filename);
    }
    void load_vars(string filename){
        //cout << "loading vars from " << filename << endl;
        ifstream fs(filename.c_str());
        string line;
        boost::char_separator<char> sep(":");
        typedef boost::tokenizer<boost::char_separator<char> > t_tokenizer;
        t_tokenizer tok(line, sep);
        while (true) {
            getline(fs, line);
            if (!fs.good())
                break;
            t_tokenizer tok(line, sep);
            t_tokenizer::iterator beg = tok.begin(); 
            string name = *beg;
            ++beg;
            T value = boost::lexical_cast<T>(*beg); 
            //cout << "found " << name << " with value " << value << endl;
            set_value(name,value);
        }
    }
    T get_value(string name){
        return vars[name];
    }
    void set_value(string name, T value){
        vars[name] = value;
    }
};
