#include "stdheader.h"

template <class T>
class VarServer{
private:
    map<string,T> vars;
    string filename;
public:
    VarServer(string _filename):filename(_filename){
        load_vars();
    }
    void load_vars(){
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
        //fs.close();
    }
    void save_vars(){
        //cout << "saving vars to " << filename << endl;
        ofstream fs(filename.c_str());
        typename map<string,T>::iterator itr;
        for(itr = vars.begin(); itr != vars.end(); ++itr){
            fs << itr->first << ":" << itr->second << endl;
        }
        //fs.close();
    }
    T get_value(string name){
        return vars[name];
    }
    void set_value(string name, T value){
        //cout << name << ": " << vars[name] << "->" <<  value << endl;
        vars[name] = value;
    }
};
