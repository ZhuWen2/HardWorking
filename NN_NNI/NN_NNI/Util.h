#pragma once
#ifndef UTIL_H
#define UTIL_H

#include <vector>

/**
 * 工具类
 */
class Util
{
public:
    // 获得txt文件中准备的数据
    std::vector<double> getFileData(const char* fileName);
};

#endif // UTIL_H

