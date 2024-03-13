#include "Util.h"
#include <string>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <vector>

using namespace std;

vector<double> Util::getFileData(const char* fileName) {
    vector<double> res;

    ifstream input(fileName);
    if (!input) {
        return res;
    }

    string buff;
    while (getline(input, buff)) {
        char* datas = (char*)buff.c_str();
        const char* spilt = " ";
        // strtok�ַ�����ֺ���
        char* data = strtok(datas, spilt);

        while (data != NULL) {
            // atof��stdlibͷ�ļ���ת���ַ���Ϊ���ֵĺ���
            res.push_back(atof(data));
            // NULL������ϴ�û�����ط�������
            data = strtok(NULL, spilt);
        }
    }

    input.close();
    return res;
}
