using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class findMinMaxGPU : MonoBehaviour
{

    public List<float> valuesArr = new List<float>();

    public ComputeShader computeShader;
    private ComputeBuffer valuesArrBuffer;
    private ComputeBuffer minArrBuffer;
    private ComputeBuffer minArrUintBuffer;
    private ComputeBuffer maxArrBuffer;
    private ComputeBuffer maxArrUintBuffer;
    //kernel
    private int findMinValueKernel;
    private int findMaxValueKernel;
    private int workgroupSize;

    // Start is called before the first frame update
    void Start()
    {
        //set value array
        setArrayValue();
        //find kernelID
        findKernelID();
        //assign buffer to kernel
        initBuffer();
        setBuffer();

       
    }

    void setArrayValue()
    {

        for(int i = 0; i < valuesArr.Count; i++)
        {
            valuesArr[i] = Random.Range(-100f, 50f);
            //valuesArr[i] = -i;
        }

        workgroupSize = Mathf.CeilToInt(valuesArr.Count / 64f);
    }
    void findKernelID()
    {
        findMinValueKernel = computeShader.FindKernel("findMinValueKernel");
        findMaxValueKernel = computeShader.FindKernel("findMaxValueKernel");



    }
    void initBuffer()
    {
        valuesArrBuffer = new ComputeBuffer(valuesArr.Count, sizeof(float));
        minArrBuffer = new ComputeBuffer(1, sizeof(float));
        maxArrBuffer = new ComputeBuffer(1, sizeof(float));

        minArrUintBuffer = new ComputeBuffer(1, sizeof(int));
        maxArrUintBuffer = new ComputeBuffer(1, sizeof(int));

        List<float> tmp = new List<float>();
        List<uint> tmp1 = new List<uint>();
        tmp.Add(0.0f);
        tmp1.Add(0);

        valuesArrBuffer.SetData(valuesArr);
        //minArrBuffer.SetData(tmp);
        //maxArrBuffer.SetData(tmp);
        //minArrUintBuffer.SetData(tmp1);
        //maxArrUintBuffer.SetData(tmp1);
    }
    void setBuffer()
    {
        computeShader.SetInt("numCount", valuesArr.Count);

        computeShader.SetBuffer(findMinValueKernel, "valuesArrBuffer", valuesArrBuffer);
        computeShader.SetBuffer(findMinValueKernel, "minArrBuffer", minArrBuffer);
        computeShader.SetBuffer(findMinValueKernel, "minArrUintBuffer", minArrUintBuffer);

        computeShader.SetBuffer(findMaxValueKernel, "valuesArrBuffer", valuesArrBuffer);
        computeShader.SetBuffer(findMaxValueKernel, "maxArrBuffer", maxArrBuffer);
        computeShader.SetBuffer(findMaxValueKernel, "maxArrUintBuffer", maxArrUintBuffer);

    }
    // Update is called once per frame
    void Update()
    {
        //dispatch compute shader
        computeShader.Dispatch(findMinValueKernel, workgroupSize, 1, 1);
        computeShader.Dispatch(findMaxValueKernel, workgroupSize, 1, 1);
        //get result

        float[] data1 = new float[1];
        float[] data2 = new float[1];
        minArrBuffer.GetData(data1);
        maxArrBuffer.GetData(data2);


        print("Min result :: " + valuesArr.Min());
        print("Max result :: " + valuesArr.Max());
        print("GPU result min:: " + data1[0]);
        print("GPU result max:: " + data2[0]);
    }

    private void OnDestroy()
    {
        if (this.enabled)
        {
            valuesArrBuffer.Dispose();
            minArrBuffer.Dispose();
            minArrUintBuffer.Dispose();
            maxArrBuffer.Dispose();
            maxArrUintBuffer.Dispose();
        }
    }

}
