#pragma once
#ifndef UTIL_H
#define UTIL_H

#include <vector>

/**
 * ������
 */
class Util
{
public:
    // ���txt�ļ���׼��������
    std::vector<double> getFileData(const char* fileName);
};

#endif // UTIL_H

