#pragma once
#ifndef BPNET_H
#define BPNET_H

#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>

#define INNODE 3     // ��������
#define HIDENODE 10   // ���������
#define OUTNODE 1    // ��������
#define LEARNINGRATE 0.9   // ѧϰ���ʣ�ע�⣺Խ����ȻԽ�� Ҳ�������ϴ�

/**
 * �����ڵ�
 */
typedef struct inputNode {
    double value;  // ����ֵ
    std::vector<double> weight  // ����㵥���ڵ����һ��ÿ���ڵ�ļ�Ȩֵ
        , wDeltaSum;  // ������Ȩ�Ĳ�ͬ������
}InputNode;

/**
 * �����ڵ�
 */
typedef struct outputNode {
    double o_value  // �ڵ�����ֵ ����ƫ���뼤������ֵ
        , rightout     // ��ȷ���ֵ
        , bias         // ƫ���� ÿ���ڵ�ֻ��һ��
        , bDeltaSum;   // ���򴫲�ʱ ����������ƫ������Ҫ�ı��ֵ ��Ϊ�ж������������sum
}OutputNode;

/**
 * ������ڵ�
 */
typedef struct hiddenNode {
    double o_value   // �ڵ�����ֵ ����ƫ���뼤������ֵ
        , bias           // ƫ���� ÿ���ڵ�ֻ��һ��
        , bDeltaSum;     // ���򴫲�ʱ ����������ƫ������Ҫ�ı��ֵ ��Ϊ�ж������������sum
    std::vector<double> weight   // ���ز㵥���ڵ����һ��ÿ���ڵ�ļ�Ȩֵ
        , wDeltaSum;     // ������Ȩ�Ĳ�ͬ������
}HiddenNode;

/**
 * ��������
 */
typedef struct sample {
    std::vector<double> in   // �����value�ĵ����� ����������������ڵ������������ÿ���ڵ��valueֵ ����һ������������ һ���������Ե�ֵ��
        , out;    // �����rightout�ĵ����� ���������Ҳ��������ڵ������������ÿ���ڵ��rightoutֵ ����һ���������� Ӧ��������Ե���ȷֵ��
}Sample;

/**
 * BP������
 */
class BpNet {
public:
    BpNet();     // ���캯�� ������ʼ����Ȩ��ƫ��
    void fp();   // ��������ǰ�򴫲�
    void bp();   // �����������򴫲�
    void doTraining(std::vector<Sample> sampleGroup, double threshold, int mostTimes);   // ѵ�������� weight, bias��
    void afterTrainTest(std::vector<Sample>& testGroup);   // ������ѧϰ�����Ԥ��
    void setInValue(std::vector<double> inValue);         // ����ѧϰ��������
    void setOutRightValue(std::vector<double> outRightValue);    // ����ѧϰ�������

public://���ó�public�Ͳ���get��set�鷳
    double error;   //�����
    InputNode* inputLayer[INNODE];    // ����㣨�κ�ģ�Ͷ�ֻ��һ�㣩
    OutputNode* outputLayer[OUTNODE]; // ����㣨�κ�ģ�Ͷ�ֻ��һ�㣩
    HiddenNode* hiddenLayer[HIDENODE]; // �����㣨�������ֻ��һ�����ز�����һά���� ������ж���Ƕ�ά���飩
};


#endif // BPNET_H
