using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class FindMinMaxVector3 : MonoBehaviour
{
    public List<Vector3> valuesArr = new List<Vector3>();
    public ComputeShader computeShader;


    private ComputeBuffer valuesArrBuffer;
    private ComputeBuffer minArrBuffer;
    private ComputeBuffer minArrUintXBuffer;
    private ComputeBuffer minArrUintYBuffer;
    private ComputeBuffer minArrUintZBuffer;

    private ComputeBuffer maxArrBuffer;
    private ComputeBuffer maxArrUintXBuffer;
    private ComputeBuffer maxArrUintYBuffer;
    private ComputeBuffer maxArrUintZBuffer;
    //kernel
    private int findMinValueKernel;
    private int findMaxValueKernel;
    private int workgroupSize;

    private Vector3 minVec3;
    private Vector3 maxVec3;

    void Start()
    {
        //set value array
        setArrayValue();
        findingMinMax();
        //find kernelID
        findKernelID();
        //assign buffer to kernel
        initBuffer();
        setBuffer();

        
    }

    void setArrayValue()
    {

        for (int i = 0; i < valuesArr.Count; i++)
        {
            Vector3 tmp = Vector3.zero;
            tmp.x = Random.Range(1f, 80f);
            tmp.y = Random.Range(1f, 80f);
            tmp.z = Random.Range(1f, 80f);

            valuesArr[i] = tmp;
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
        valuesArrBuffer = new ComputeBuffer(valuesArr.Count, sizeof(float) * 3);
        minArrBuffer = new ComputeBuffer(1, sizeof(float) * 3);
        maxArrBuffer = new ComputeBuffer(1, sizeof(float) * 3);

        minArrUintXBuffer = new ComputeBuffer(1, sizeof(uint));
        minArrUintYBuffer = new ComputeBuffer(1, sizeof(uint));
        minArrUintZBuffer = new ComputeBuffer(1, sizeof(uint));

        maxArrUintXBuffer = new ComputeBuffer(1, sizeof(uint));
        maxArrUintYBuffer = new ComputeBuffer(1, sizeof(uint));
        maxArrUintZBuffer = new ComputeBuffer(1, sizeof(uint));

        valuesArrBuffer.SetData(valuesArr);
       
    }
    void setBuffer()
    {
        computeShader.SetInt("numCount", valuesArr.Count);

        computeShader.SetBuffer(findMinValueKernel, "valuesArrBuffer", valuesArrBuffer);
        computeShader.SetBuffer(findMinValueKernel, "minArrBuffer", minArrBuffer);
        computeShader.SetBuffer(findMinValueKernel, "minArrUintXBuffer", minArrUintXBuffer);
        computeShader.SetBuffer(findMinValueKernel, "minArrUintYBuffer", minArrUintYBuffer);
        computeShader.SetBuffer(findMinValueKernel, "minArrUintZBuffer", minArrUintZBuffer);

        computeShader.SetBuffer(findMaxValueKernel, "valuesArrBuffer", valuesArrBuffer);
        computeShader.SetBuffer(findMaxValueKernel, "maxArrBuffer", maxArrBuffer);
        computeShader.SetBuffer(findMaxValueKernel, "maxArrUintXBuffer", maxArrUintXBuffer);
        computeShader.SetBuffer(findMaxValueKernel, "maxArrUintYBuffer", maxArrUintYBuffer);
        computeShader.SetBuffer(findMaxValueKernel, "maxArrUintZBuffer", maxArrUintZBuffer);

    }
    void findingMinMax()
    {
        minVec3 = valuesArr[0];
        maxVec3 = valuesArr[0];
        foreach (Vector3 vector in valuesArr)
        {
            

            // Update minimum values
            minVec3.x = Mathf.Min(minVec3.x, vector.x);
            minVec3.y = Mathf.Min(minVec3.y, vector.y);
            minVec3.z = Mathf.Min(minVec3.z, vector.z);

            // Update maximum values
            maxVec3.x = Mathf.Max(maxVec3.x, vector.x);
            maxVec3.y = Mathf.Max(maxVec3.y, vector.y);
            maxVec3.z = Mathf.Max(maxVec3.z, vector.z);
        }

        print("Min result :: " + minVec3.ToString());
        print("Max result :: " + maxVec3.ToString());

    }

    // Update is called once per frame
    void Update()
    {
        computeShader.Dispatch(findMinValueKernel, workgroupSize, 1, 1);
        computeShader.Dispatch(findMaxValueKernel, workgroupSize, 1, 1);

        Vector3[] data1 = new Vector3[1];
        Vector3[] data2 = new Vector3[1];
        minArrBuffer.GetData(data1);
        maxArrBuffer.GetData(data2);

        print("GPU result min:: " + data1[0].ToString());
        print("GPU result max:: " + data2[0].ToString());


    }

    private void OnDestroy()
    {
        if (this.enabled)
        {
            valuesArrBuffer.Dispose();
            minArrBuffer.Dispose();
            minArrUintXBuffer.Dispose();
            minArrUintYBuffer.Dispose();
            minArrUintZBuffer.Dispose();
            maxArrBuffer.Dispose();
            maxArrUintXBuffer.Dispose();
            maxArrUintYBuffer.Dispose();
            maxArrUintZBuffer.Dispose();



        }
    }
   
}
