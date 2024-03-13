#include "BpNet.h"
#include "Util.h"

using namespace std;

void getInput(double& threshold, int& mostTimes);  // �������ķ�ֵ������С
vector<Sample> getTrianData();           // ���ļ���ȡѵ������ û��ȡ��ֱ���˳�
vector<Sample> getTestData();            // ���ļ���ȡ�������� û��ȡ��ֱ���˳�
void showTest(vector<Sample>testGroup);  // ����������ݵĽ��

int main() {
    // ׼����������
    BpNet bpNet;
    vector<Sample> sampleGroup = getTrianData();
    vector<Sample> testGroup = getTestData();
    double threshold;   // �趨�ķ�ֵ
    int mostTimes;      // ���ѵ������

    // ��ȡ���� ����ʾ�����Ѿ�¼��
    getInput(threshold, mostTimes);

    // ����ѵ��
    bpNet.doTraining(sampleGroup, threshold, mostTimes);

    // ѵ�������¼������� ����Ĳ���������
    bpNet.afterTrainTest(testGroup);
    // ��ӡ��ǰ¼�����ݵĲ��Խ��
    showTest(testGroup);

    return 0;
}

void getInput(double& threshold, int& mostTimes) {
    cout << "ѵ�������������Ѵ��ļ�����" << endl << endl;
    cout << "������XORѵ�������";   //0.0001���
    cin >> threshold;
    cout << "������XORѵ����������";
    cin >> mostTimes;
}

void showTest(vector<Sample> testGroup) {
    // ������Խ��
    cout << "ϵͳ��������:" << endl;
    for (int i = 0; i < testGroup.size(); i++) {
        for (int j = 0; j < testGroup[i].in.size(); j++) {
            cout << testGroup[i].in[j] << "\t";
        }

        cout << "-- XORѵ����� :";
        for (int j = 0; j < testGroup[i].out.size(); j++) {
            cout << testGroup[i].out[j] << "\t";
        }
        cout << endl;
    }

    cout << endl << endl;
    system("pause");
}


vector<Sample> getTestData() {
    Util util;
    vector<double> testData = util.getFileData("test.txt");
    if (testData.size() == 0) {
        cout << "�����������ʧ�ܣ�" << endl;
        exit(0);
    }

    int groups = testData.size() / INNODE;
    
    // ������������
    //Sample testInOut[groups];
    vector<Sample> testInOut(groups);

    for (int i = 0, index = 0; i < groups; i++) {
        for (int j = 0; j < INNODE; j++) {
            testInOut[i].in.push_back(testData[index++]);
        }
    }

    // ��ʼ������
    return vector<Sample>(testInOut.begin(), testInOut.begin() + groups);
}

vector<Sample> getTrianData() {
    Util util;
    vector<double> trainData = util.getFileData("data.txt");
    if (trainData.size() == 0) {
        cout << "����ѵ������ʧ�ܣ�" << endl;
        exit(0);
    }

    int groups = trainData.size() / (INNODE+1);
    // ������������
    //Sample trainInOut[groups];
    vector<Sample> trainInOut(groups);
    // ��vector���ø�����Sample
    for (int i = 0, index = 0; i < groups; i++) {
        for (int j = 0; j < INNODE + 1; j++) {
            if (j % (INNODE + 1) != INNODE) {
                trainInOut[i].in.push_back(trainData[index++]);
            }
            else {
                trainInOut[i].out.push_back(trainData[index++]);
            }
        }
    }

    // ��ʼ��¼��ĸ�����
    return vector<Sample>(trainInOut.begin(), trainInOut.begin() + groups);
}
