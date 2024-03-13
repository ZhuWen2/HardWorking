#include "BpNet.h"

using namespace std;

/**
 * ����-0.01~0.01�������
 */
inline double getRandom() {
    return (((2.0 * (double)rand() / RAND_MAX) - 1));
}

/**
 * sigmoid ����������� Ҫ��֤���� ��ֻ��һ��������
 */
inline double sigmoid(double x) {
    // һ��bp��������Ļ����øú���
    double ans = 1 / (1 + exp(-x));
    return ans;
}


/**
 * ��ʼ��������Ȩ����ƫ�Ƹ���ֵ��
 */
BpNet::BpNet() {
    srand((unsigned)time(NULL));
    // error��ʼֵ��ֻҪ�ܱ�֤���ڷ�ֵ����ѵ���Ϳ���
    error = 90000.f;

    /*
     * ��ʼ�������ÿ���ڵ����һ��ÿ���ڵ�ļ�Ȩ
     */
    for (int i = 0; i < INNODE; i++) {
        inputLayer[i] = new InputNode();
        for (int j = 0; j < HIDENODE; j++) {
            inputLayer[i]->weight.push_back(getRandom());
            inputLayer[i]->wDeltaSum.push_back(0.f);
        }
    }

    /*
     * ��ʼ�����ز�ÿ���ڵ����һ��ÿ���ڵ�ļ�Ȩ
     * ��ʼ�����ز�ÿ���ڵ��ƫ��
     */
    for (int i = 0; i < HIDENODE; i++) {
        hiddenLayer[i] = new HiddenNode();
        hiddenLayer[i]->bias = getRandom();

        // ��ʼ����Ȩ
        for (int j = 0; j < OUTNODE; j++) {
            hiddenLayer[i]->weight.push_back(getRandom());
            hiddenLayer[i]->wDeltaSum.push_back(0.f);
        }
    }

    /*
     * ��ʼ�������ÿ���ڵ��ƫ��
     */
    for (int i = 0; i < OUTNODE; i++) {
        outputLayer[i] = new OutputNode();
        outputLayer[i]->bias = getRandom();
    }
}


/**
 * ���򴫲� ��ȡһ�����������뵽����Ľ��
 */
void BpNet::fp() {
    /*
     * ���ز���������ȡ����
     */
     // �������ز�ڵ�
    for (int i = 0; i < HIDENODE; i++) {
        double sum = 0.f;

        // ���������ÿ���ڵ�
        for (int j = 0; j < INNODE; j++) {
            sum += inputLayer[j]->value * inputLayer[j]->weight[i];
        }

        // ����ƫ��
        sum += hiddenLayer[i]->bias;
        // ���ü���� ����o_value
        hiddenLayer[i]->o_value = sigmoid(sum);
    }

    /*
     * ����������ز��ȡ����
     */
     // ���������ڵ�
    for (int i = 0; i < OUTNODE; i++) {
        double sum = 0.f;

        // �������ز�ڵ�
        for (int j = 0; j < HIDENODE; j++) {
            sum += hiddenLayer[j]->o_value * hiddenLayer[j]->weight[i];
        }

        sum += outputLayer[i]->bias;
        outputLayer[i]->o_value = sigmoid(sum);
    }
}


/**
 * ���򴫲� ��������ٷ���
 *
 * �÷���Ŀ���Ƿ��أ����������ȨӦ�ñ仯ֵ�ĺ͡�wDeltaSum�����������ƫ��Ӧ�ñ仯ֵ�ĺ͡�bDeltaSum��
 * ��ѵ��ʱ�����������仯ֵ����ƽ��ֵ �ø�ƽ��ֵ�޸ļ�Ȩ��ƫ��
 *
 */
void BpNet::bp() {
    /*
     * �����ֵerror
     */
    for (int i = 0; i < OUTNODE; i++) {
        double tmpe = fabs(outputLayer[i]->o_value - outputLayer[i]->rightout);
        // ������� ���������һ����ʽ
        error += tmpe * tmpe / 2;
    }


    /*
     * �������ƫ�Ƶı仯ֵ
     */
    for (int i = 0; i < OUTNODE; i++) {
        // ƫ��Ӧ�ñ仯��ֵ ����b2��ʽ
        double bDelta = (-1) * (outputLayer[i]->rightout - outputLayer[i]->o_value) * outputLayer[i]->o_value * (1 - outputLayer[i]->o_value);
        outputLayer[i]->bDeltaSum += bDelta;
    }

    /*
     * ���������Ȩ�ı仯ֵ
     */
    for (int i = 0; i < HIDENODE; i++) {
        for (int j = 0; j < OUTNODE; j++) {
            // ��ȨӦ�ñ仯��ֵ ����w9��ʽ
            double wDelta = (-1) * (outputLayer[j]->rightout - outputLayer[j]->o_value) * outputLayer[j]->o_value * (1 - outputLayer[j]->o_value) * hiddenLayer[i]->o_value;
            hiddenLayer[i]->wDeltaSum[j] += wDelta;
        }
    }

    /*
     * �����ز�ƫ��
     */
    for (int i = 0; i < HIDENODE; i++) {
        double sum = 0;   // ��Ϊ�Ǳ��������ڵ� ������ȷ���ж��ٸ�����ڵ� ����b1��ʽ�ĵ�һ������ʽ
        for (int j = 0; j < OUTNODE; j++) {
            sum += (-1) * (outputLayer[j]->rightout - outputLayer[j]->o_value) * outputLayer[j]->o_value * (1 - outputLayer[j]->o_value) * hiddenLayer[i]->weight[j];
        }
        // ���չ�ʽb1
        hiddenLayer[i]->bDeltaSum += (sum * hiddenLayer[i]->o_value * (1 - hiddenLayer[i]->o_value));
    }

    /*
     * �����������ز�ļ�Ȩ�仯
     */
    for (int i = 0; i < INNODE; i++) {
        // �ӹ�ʽb1��w1���Կ��� ������ʽ���й���ʽ �����ⲿ�ִ�����ͬ
        double sum = 0;
        for (int j = 0; j < HIDENODE; j++) {
            for (int k = 0; k < OUTNODE; k++) {
                sum += (-1) * (outputLayer[k]->rightout - outputLayer[k]->o_value) * outputLayer[k]->o_value * (1 - outputLayer[k]->o_value) * hiddenLayer[j]->weight[k];
            }
            // ���չ�ʽw1
            inputLayer[i]->wDeltaSum[j] += (sum * hiddenLayer[j]->o_value * (1 - hiddenLayer[j]->o_value) * inputLayer[i]->value);
        }
    }

}


/**
 * ����ѵ�� ������������޸ĵĹ�ʽ
 */
void BpNet::doTraining(vector<Sample> sampleGroup, double threshold, int mostTimes) {
    int sampleNum = sampleGroup.size();
    int trainTimes = 0;
    bool isSuccess = true;

    while (error >= threshold) {
        // �ж��Ƿ񳬹����ѵ������
        if (trainTimes > mostTimes) {
            isSuccess = false;
            break;
        }

        cout << "ѵ������:" << trainTimes++ << "\t\t" << "��ǰ���: " << error << endl;
        error = 0.f;

        // ��ʼ��������Ȩ��delta��
        for (int i = 0; i < INNODE; i++) {
            inputLayer[i]->wDeltaSum.assign(inputLayer[i]->wDeltaSum.size(), 0.f);
        }

        // ��ʼ�����ز��Ȩ��ƫ�Ƶ�delta��
        for (int i = 0; i < HIDENODE; i++) {
            hiddenLayer[i]->wDeltaSum.assign(hiddenLayer[i]->wDeltaSum.size(), 0.f);
            hiddenLayer[i]->bDeltaSum = 0.f;
        }

        // ��ʼ��������ƫ�ƺ�
        for (int i = 0; i < OUTNODE; i++) {
            outputLayer[i]->bDeltaSum = 0.f;
        }

        // ������������ĵ����뷴��
        for (int iter = 0; iter < sampleNum; iter++) {
            setInValue(sampleGroup[iter].in);
            setOutRightValue(sampleGroup[iter].out);

            fp();
            bp();
        }

        // �޸������ļ�Ȩ
        for (int i = 0; i < INNODE; i++) {
            for (int j = 0; j < HIDENODE; j++) {
                //ÿһ����Ȩ�ĺͶ������������ۻ��� ����Ҫ����������
                inputLayer[i]->weight[j] -= LEARNINGRATE * inputLayer[i]->wDeltaSum[j] / sampleNum;
            }
        }

        // �޸����ز�ļ�Ȩ��ƫ��
        for (int i = 0; i < HIDENODE; i++) {
            // �޸�ÿ���ڵ��ƫ�� ��Ϊһ���ڵ��һ��ƫ�� ���Բ����ڽڵ����ٱ���
            hiddenLayer[i]->bias -= LEARNINGRATE * hiddenLayer[i]->bDeltaSum / sampleNum;

            // �޸�ÿ���ڵ�ĸ�����Ȩ��ֵ
            for (int j = 0; j < OUTNODE; j++) {
                hiddenLayer[i]->weight[j] -= LEARNINGRATE * hiddenLayer[i]->wDeltaSum[j] / sampleNum;
            }
        }

        //�޸�������ƫ��
        for (int i = 0; i < OUTNODE; i++) {
            outputLayer[i]->bias -= LEARNINGRATE * outputLayer[i]->bDeltaSum / sampleNum;
        }
    }

    if (isSuccess) {
        cout << endl << "ѵ���ɹ�!!!" << "\t\t" << "�������: " << error << endl << endl;
    }
    else {
        cout << endl << "ѵ��ʧ��! ����������!" << "\t\t" << "�������: " << error << endl << endl;
    }

}


/**
 * ѵ������в���ʹ��
 */
void BpNet::afterTrainTest(vector<Sample>& testGroup) {
    int testNum = testGroup.size();

    for (int iter = 0; iter < testNum; iter++) {
        // ������������
        testGroup[iter].out.clear();
        setInValue(testGroup[iter].in);

        // �����ز��������ȡ����
        for (int i = 0; i < HIDENODE; i++) {
            double sum = 0.f;
            for (int j = 0; j < INNODE; j++) {
                sum += inputLayer[j]->value * inputLayer[j]->weight[i];
            }

            sum += hiddenLayer[i]->bias;
            hiddenLayer[i]->o_value = sigmoid(sum);
        }

        // ���������ز��ȡ����
        for (int i = 0; i < OUTNODE; i++) {
            double sum = 0.f;
            for (int j = 0; j < HIDENODE; j++) {
                sum += hiddenLayer[j]->o_value * hiddenLayer[j]->weight[i];
            }

            sum += outputLayer[i]->bias;
            outputLayer[i]->o_value = sigmoid(sum);

            // ���������ֵ
            testGroup[iter].out.push_back(outputLayer[i]->o_value);
        }
    }
}


/**
 * �������ÿ���ڵ���������ֵ ÿ����������ѵ��ʱ��Ҫ����
 */
void BpNet::setInValue(vector<double> sampleIn) {
    // ��Ӧһ������ �����ÿ���ڵ������ֵ
    for (int i = 0; i < INNODE; i++) {
        inputLayer[i]->value = sampleIn[i];
    }
}

/**
 * �������ÿ���ڵ�������ȷֵ ÿ����������ѵ��ʱ��Ҫ����
 */
void BpNet::setOutRightValue(vector<double> sampleOut) {
    // ��Ӧһ������ ������ÿ���ڵ����ȷֵ
    for (int i = 0; i < OUTNODE; i++) {
        outputLayer[i]->rightout = sampleOut[i];
    }
}
