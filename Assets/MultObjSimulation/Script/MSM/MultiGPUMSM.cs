using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using System.Linq;
using UnityEngine.Rendering;
using PBD;
using Octree;
using ExporterImporter;

public class MultiGPUMSM : MonoBehaviour
{
    public enum MyModel
    {
        IcoSphere_low,
        Torus,
        Bunny,
        Armadillo,
    };


    [Header("Deformable model")]
    public int number_object = 1;
    [Header("Random Range")]
    Vector3 rangeMin = new Vector3(-10f, 0f, 0f);
    Vector3 rangeMax = new Vector3(10f, 10f, 20f);

    [Header("3D model")]
    public MyModel model;

    [Header("Import CSV")]
    public string csv_file = "object_positions.csv";

    [HideInInspector]
    private string modelName;

    [Header("Obj Parameters")]
    public float dt = 0.005f; // have to devide by 20
    public Vector3 gravity = new Vector3(0f, -9.81f, 0f);

    public int speed = 1;

    [Header("Geometric Parameters")]
    public bool useInteriorSpring = false;


    [Header("Collision")]
    public GameObject[] collidableObject;

    [Header("GPU Paramenter")]
    public ComputeShader computeShader;
    public ComputeShader OctreeAabbCS;

    [Header("Rendering")]
    public Shader renderingShader;
    public Color matColor;

    [HideInInspector]
    private GameObject[] deformableObjectList;
    private GPUMSM[] deformableGPUMSM;

    GPUMSM gpumsmScript;

    [Header("Debug Mode")]
    public bool debugModeData = true;
    public bool debugModeL0 = true;
    public bool debugModeL1 = true;
    public bool debugModeL2 = true;
   

    private Vector2Int[] indicies;
    private int nodeCount;

    private readonly int octree_size = 73;

    // kernel IDs
    private int bindPositionsKernel;
    private int findBBMinMaxKernel;
    private int implementOctreeKernel;
    private int checkCollisionL0Kernel;
    private int checkCollisionL1Kernel;
    private int checkCollisionL2Kernel;

    // Compute Buffer
    private ComputeBuffer positionsBuffer;
    private ComputeBuffer globalPositionsBuffer;
    private ComputeBuffer bbMinMaxBuffer;
    private ComputeBuffer rangeObjIndexBuffer;
    private ComputeBuffer bbOctreeBuffer;
    private ComputeBuffer collisionPairBuffer;
    private ComputeBuffer pairIndexL0Buffer;
    private ComputeBuffer pairIndexL1Buffer;
    private ComputeBuffer pairIndexL2Buffer;
    private ComputeBuffer collisionResultsL0Buffer;
    private ComputeBuffer collisionResultsL1Buffer;
    private ComputeBuffer collisionResultsL2Buffer;
    // data
    private OctreeData[] bbOctree;
    private PairData[] pairIndexL0;
    private PairData[] pairIndexL1;
    private PairData[] pairIndexL2;

    // collision pair
    private List<int> collidablePairIndexL0 = new List<int>();
    private List<int> collidablePairIndexL1 = new List<int>();
    private List<int> collidablePairIndexL2 = new List<int>();
    
    private int[] collisionPairResults;
    private int[] collisionresultsL0;
    private int[] collisionresultsL1;
    private int[] collisionresultsL2;

    void Start()
    {
        
        SelectModelName();
        addDeformableObjectList();
        AddOctreePairIndex();

        FindKernelIDs();
        SetupComputeBuffer();
        SetupComputeShader();
    }

    void SelectModelName()
    {
        switch (model)
        {
            case MyModel.IcoSphere_low: modelName = "icosphere_low.1"; break;
            case MyModel.Torus: modelName = "torus.1"; break;
            case MyModel.Bunny: modelName = "bunny.1"; break;
            case MyModel.Armadillo: modelName = "Armadillo.1"; break;
        }
    }
    void addDeformableObjectList()
    {
        deformableObjectList = new GameObject[number_object];
        HashSet<Vector3> generatedPositions = new HashSet<Vector3>();
        deformableGPUMSM = new GPUMSM[number_object];
        indicies = new Vector2Int[number_object];
        int st_index = 0;

        List<List<string>> csvData = ExporterAndImporter.ReadCSVFile(csv_file);
        Vector3 position = new Vector3();

        for (int i = 0; i < number_object; i++)
        {
            
            deformableObjectList[i] = new GameObject("Deformable Object " + i);
            deformableObjectList[i].transform.SetParent(this.transform);
           
            //set position of the object 1). randomize 2).set the coord
            Vector3 randomPosition;

            do
            {
                // Generate random position within the specified range
                float x = UnityEngine.Random.Range(rangeMin.x, rangeMax.x);
                float y = UnityEngine.Random.Range(rangeMin.y, rangeMax.y);
                float z = UnityEngine.Random.Range(rangeMin.z, rangeMax.z);

                randomPosition = new Vector3(x, y, z);
            } while (generatedPositions.Contains(randomPosition));
            //deformableObjectList[i].transform.position = randomPosition;

            List<string> row = csvData[i];
            for (int j = 1; j < row.Count; j++)
            {
                float x = float.Parse(row[0]);
                float y = float.Parse(row[1]);
                float z = float.Parse(row[2]);

                position = new Vector3(x, y, z);
            }
            deformableObjectList[i].transform.position = position;

            
            deformableObjectList[i].transform.localScale = transform.localScale;
            deformableObjectList[i].transform.rotation = transform.rotation;

            if (deformableObjectList[i] != null)
            {
                gpumsmScript = deformableObjectList[i].AddComponent<GPUMSM>();
                gpumsmScript.SetCoefficient( dt, gravity, speed);
                gpumsmScript.SetMeshData(modelName, useInteriorSpring);
                Shader tmp = Instantiate(renderingShader);
                ComputeShader tmpCS = Instantiate(computeShader);
                Color randomColor = new Color(
                    UnityEngine.Random.value,
                    UnityEngine.Random.value,
                    UnityEngine.Random.value);
                //gpumsmScript.SetRenderer(tmp, matColor);
                gpumsmScript.SetRenderer(tmpCS, tmp, randomColor);
                gpumsmScript.SetCollidableObj(collidableObject);
                deformableGPUMSM[i] = gpumsmScript;


                deformableGPUMSM[i].StartObj();

                nodeCount = deformableGPUMSM[i].getNodeCount();

                indicies[i] = new Vector2Int(st_index, st_index + nodeCount);
                st_index += nodeCount;
            }

        }
    }

    int calculateStartIndex(int object_index, int level)
    {
        if (level == 1) return object_index * 73 + 1;
        if (level == 2) return object_index * 73 + 9;
        return object_index * 73;
    }

    int calculateEndIndex(int object_index, int level)
    {
        if (level == 1) return object_index * 73 + 8;
        if (level == 2) return object_index * 73 + 73;
        return object_index * 73;
    }

    private void AddOctreePairIndex()
    {
        // index pair
        List<PairIndex> indexPairsL0 = new List<PairIndex>();
        List<PairIndex> indexPairsL1 = new List<PairIndex>();
        List<PairIndex> indexPairsL2 = new List<PairIndex>();

        for (int i = 0; i < number_object; i++)
        {
            var l0_st_idx = calculateStartIndex(i, 0);
            var l1_st_idx = calculateStartIndex(i, 1);
            var l2_st_idx = calculateStartIndex(i, 2);

            var l0_end_idx = calculateEndIndex(i, 0);
            var l1_end_idx = calculateEndIndex(i, 1);
            var l2_end_idx = calculateEndIndex(i, 2);

            for (int j = 0; j < number_object; j++)
            {
                if (i == j) break;

                var j_l0_st_idx = calculateStartIndex(j, 0);
                var j_l0_end_idx = calculateStartIndex(j, 0);

                var j_l1_st_idx = calculateStartIndex(j, 1);
                var j_l2_st_idx = calculateStartIndex(j, 2);

                var j_l1_end_idx = calculateEndIndex(j, 1);
                var j_l2_end_idx = calculateEndIndex(j, 2);


                for (int n = l0_st_idx; n <= l0_end_idx; n++)
                {
                    for (int m = j_l0_st_idx; m <= j_l0_end_idx; m++)
                    {
                        List<int> indices = new List<int>();
                        indices.Add(n);
                        indices.Add(m);
                        PairIndex pair = new PairIndex(indices);
                        indexPairsL0.Add(pair);
                        //pair_datas.Add("pair : " + n.ToString() + ":" + m.ToString());
                    }
                }
                for (int n = l1_st_idx; n <= l1_end_idx; n++)
                {
                    for (int m = j_l1_st_idx; m <= j_l1_end_idx; m++)
                    {
                        //pair_datas.Add("pair : " + n.ToString() + ":" + m.ToString());

                        List<int> indices = new List<int>();
                        indices.Add(n);
                        indices.Add(m);
                        PairIndex pair = new PairIndex(indices);
                        indexPairsL1.Add(pair);

                        //print($" m n {m} {n}");
                    }
                }
                for (int n = l2_st_idx; n < l2_end_idx; n++)
                {
                    for (int m = j_l2_st_idx; m < j_l2_end_idx; m++)
                    {
                        List<int> indices = new List<int>();
                        indices.Add(n);
                        indices.Add(m);
                        PairIndex pair = new PairIndex(indices);
                        indexPairsL2.Add(pair);

                        //pair_datas.Add("pair : " + n.ToString() + ":" + m.ToString());
                    }
                }
            }
        }

        UpdatePairIndexData(indexPairsL0, indexPairsL1, indexPairsL2);
    }

    // Update pair array
    private void UpdatePairIndexData(List<PairIndex> pairIdxL0, List<PairIndex> pairIdxL1, List<PairIndex> pairIdxL2)
    {
        pairIndexL0 = new PairData[pairIdxL0.Count];
        for (int i = 0; i < pairIdxL0.Count; i++)
        {
            pairIndexL0[i] = new PairData
            {
                i1 = pairIdxL0[i].index[0],
                i2 = pairIdxL0[i].index[1]
            };
        }

        
        pairIndexL1 = new PairData[pairIdxL1.Count];
        for (int i = 0; i < pairIdxL1.Count; i++)
        {
            pairIndexL1[i] = new PairData
            {
                i1 = pairIdxL1[i].index[0],
                i2 = pairIdxL1[i].index[1]
            };
        }

        pairIndexL2 = new PairData[pairIdxL2.Count];
        for (int i = 0; i < pairIdxL2.Count; i++)
        {
            pairIndexL2[i] = new PairData
            {
                i1 = pairIdxL2[i].index[0],
                i2 = pairIdxL2[i].index[1]
            };
        }
    }

    private void FindKernelIDs()
    {
        bindPositionsKernel = OctreeAabbCS.FindKernel("BindGlobalPositions");
        findBBMinMaxKernel = OctreeAabbCS.FindKernel("FindBBMinMax");

        implementOctreeKernel = OctreeAabbCS.FindKernel("ImplementOctree");
        checkCollisionL0Kernel = OctreeAabbCS.FindKernel("CheckCollisionL0");
        checkCollisionL1Kernel = OctreeAabbCS.FindKernel("CheckCollisionL1");
        checkCollisionL2Kernel = OctreeAabbCS.FindKernel("CheckCollisionL2");
    }

    private void SetupComputeBuffer()
    {
        collisionresultsL0 = new int[pairIndexL0.Length];
        collisionresultsL1 = new int[pairIndexL1.Length];
        collisionresultsL2 = new int[pairIndexL2.Length];
        collisionPairResults = new int[number_object * octree_size];

        globalPositionsBuffer = new ComputeBuffer(nodeCount * number_object, sizeof(float) * 3);
        positionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        bbMinMaxBuffer = new ComputeBuffer(number_object, sizeof(float) * 6);

        rangeObjIndexBuffer = new ComputeBuffer(number_object, sizeof(int) * 2);
        bbOctreeBuffer = new ComputeBuffer(number_object * octree_size, sizeof(float) * 13);

        pairIndexL0Buffer = new ComputeBuffer(pairIndexL0.Length, sizeof(int) * 2);
        pairIndexL1Buffer = new ComputeBuffer(pairIndexL1.Length, sizeof(int) * 2);
        pairIndexL2Buffer = new ComputeBuffer(pairIndexL2.Length, sizeof(int) * 2);
        collisionResultsL0Buffer = new ComputeBuffer(pairIndexL0.Length, sizeof(int));
        collisionResultsL1Buffer = new ComputeBuffer(pairIndexL1.Length, sizeof(int));
        collisionResultsL2Buffer = new ComputeBuffer(pairIndexL2.Length, sizeof(int));
        collisionPairBuffer = new ComputeBuffer(number_object * octree_size, sizeof(int));
    }

    private void SetupComputeShader()
    {
        bbOctree = new OctreeData[number_object * octree_size];
        

        OctreeAabbCS.SetInt("numberObj", number_object);
        OctreeAabbCS.SetInt("nodeCount", nodeCount);

        rangeObjIndexBuffer.SetData(indicies);
        pairIndexL0Buffer.SetData(pairIndexL0);
        pairIndexL1Buffer.SetData(pairIndexL1);
        pairIndexL2Buffer.SetData(pairIndexL2);

        // BindGlobalPositions
        OctreeAabbCS.SetBuffer(bindPositionsKernel, "globalPositions", globalPositionsBuffer);

        // findBBMinMaxKernel
        OctreeAabbCS.SetBuffer(findBBMinMaxKernel, "bbMinMax", bbMinMaxBuffer);
        OctreeAabbCS.SetBuffer(findBBMinMaxKernel, "rangeObjIndex", rangeObjIndexBuffer);
        OctreeAabbCS.SetBuffer(findBBMinMaxKernel, "globalPositions", globalPositionsBuffer);

        // ImplementOctree
        OctreeAabbCS.SetBuffer(implementOctreeKernel, "bbMinMax", bbMinMaxBuffer);
        OctreeAabbCS.SetBuffer(implementOctreeKernel, "bbOctree", bbOctreeBuffer);

        // CheckCollisionLv0
        OctreeAabbCS.SetBuffer(checkCollisionL0Kernel, "bbOctree", bbOctreeBuffer); 
        OctreeAabbCS.SetBuffer(checkCollisionL0Kernel, "collisionResultL0", collisionResultsL0Buffer);
        OctreeAabbCS.SetBuffer(checkCollisionL0Kernel, "pairIndexL0", pairIndexL0Buffer);
        OctreeAabbCS.SetBuffer(checkCollisionL0Kernel, "collisionPairResult", collisionPairBuffer);

        // CheckCollisionL1
        OctreeAabbCS.SetBuffer(checkCollisionL1Kernel, "pairIndexL1", pairIndexL1Buffer);
        OctreeAabbCS.SetBuffer(checkCollisionL1Kernel, "collisionResultL1", collisionResultsL1Buffer);
        OctreeAabbCS.SetBuffer(checkCollisionL1Kernel, "bbOctree", bbOctreeBuffer);
        OctreeAabbCS.SetBuffer(checkCollisionL1Kernel, "collisionPairResult", collisionPairBuffer);

        // CheckCollisionL2
        OctreeAabbCS.SetBuffer(checkCollisionL2Kernel, "pairIndexL2", pairIndexL2Buffer);
        OctreeAabbCS.SetBuffer(checkCollisionL2Kernel, "collisionResultL2", collisionResultsL2Buffer);
        OctreeAabbCS.SetBuffer(checkCollisionL2Kernel, "bbOctree", bbOctreeBuffer);
        OctreeAabbCS.SetBuffer(checkCollisionL2Kernel, "collisionPairResult", collisionPairBuffer);

    }


    private void Update()
    {
        for (int i = 0; i < number_object; i++)
        {
            OctreeAabbCS.SetInt("objectIndex", i);
            deformableGPUMSM[i].UpdateObj();

            //if (deformableGPUMSM[i].GetPositionBuffer() != null) positionsBuffer.Release();
            positionsBuffer = deformableGPUMSM[i].GetPositionBuffer();

            //if (deformableGPUMSM[i].GetPositionBuffer() != null) positionsBuffer.Release();
            OctreeAabbCS.SetBuffer(bindPositionsKernel, "positions", positionsBuffer);


            int numGroups_Pos = Mathf.CeilToInt(nodeCount / 1024f);
            OctreeAabbCS.Dispatch(bindPositionsKernel, numGroups_Pos, 1, 1);

        }

        DispatchComputeShader();


        GetData();
    }

    private void DispatchComputeShader()
    {
        OctreeAabbCS.Dispatch(findBBMinMaxKernel, Mathf.CeilToInt(number_object / 1024f), 1, 1);
        OctreeAabbCS.Dispatch(implementOctreeKernel, Mathf.CeilToInt(number_object / 1024f)+1, 1, 1);
        OctreeAabbCS.Dispatch(checkCollisionL0Kernel, Mathf.CeilToInt(pairIndexL0.Length / 1024f)+1, 1, 1);
        OctreeAabbCS.Dispatch(checkCollisionL1Kernel, Mathf.CeilToInt(pairIndexL1.Length / 1024f)+1, 1, 1);
        OctreeAabbCS.Dispatch(checkCollisionL2Kernel, Mathf.CeilToInt(pairIndexL2.Length / 1024f)+1, 1, 1);
       
    }

    private void GetData()
    {
        if (debugModeData)
        bbOctreeBuffer.GetData(bbOctree);


        collidablePairIndexL0.Clear();
        collidablePairIndexL1.Clear();
        collidablePairIndexL2.Clear();

        if (debugModeL0)
        {
            collisionResultsL0Buffer.GetData(collisionresultsL0);

            for (int i = 0; i < collisionresultsL0.Length; i++)
            {
                if (collisionresultsL0[i] == 1)
                {
                    //print("i " + i);
                    collidablePairIndexL0.Add(i);

                }
            }
        }

        if (debugModeL1)
        {
            collisionResultsL1Buffer.GetData(collisionresultsL1);


            for (int i = 0; i < collisionresultsL1.Length; i++)
            {
                //print($"Lv1 i {i} {collisionresultsL1[i]}");
                if (collisionresultsL1[i] == 1)
                {
                    collidablePairIndexL1.Add(i);

                }

            }

        }

        if (debugModeL2)
        {
            collisionResultsL2Buffer.GetData(collisionresultsL2);

            for (int i = 0; i < collisionresultsL2.Length; i++)
            {
                //print($"Lv1 i {i} {collisionresultsL2[i]}");
                if (collisionresultsL2[i] == 1)
                {
                    collidablePairIndexL2.Add(i);

                }
            }
        }
        

        //collisionPairBuffer.GetData(collisionPairResults);
        // print($"i {collisionPairResults.Length}");

        //for (int i = 0; i < collisionPairResults.Length; i++)
        //{
            //print($"coll i {i} {collisionPairResults[i]}");

        //}
    }



    private void OnGUI()
    {
        int w = Screen.width, h = Screen.height;
        GUIStyle style = new GUIStyle();
        Rect rect = new Rect(20, 40, w, h * 2 / 100);
        style.alignment = TextAnchor.UpperLeft;
        style.fontSize = h * 2 / 50;
        style.normal.textColor = Color.yellow;


        string text = string.Format("num. Obj :: " + number_object);
        GUI.Label(rect, text, style);

    }


    private void OnDrawGizmos()
    {
        if (bbOctree != null && debugModeData)
        {
            for (int i = 0; i < bbOctree.Length; i++)
            {
                Gizmos.color = Color.red;
                for (int p = 0; p < collidablePairIndexL0.Count; p++)
                {
                    if (i == pairIndexL0[collidablePairIndexL0[p]].i1)
                    {
                        Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);
                    }

                    if (i == pairIndexL0[collidablePairIndexL0[p]].i2)
                    {
                        Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);
                    }

                }
                
                Gizmos.color = Color.green;
                for (int p = 0; p < collidablePairIndexL1.Count; p++)
                {
                    if (i == pairIndexL1[collidablePairIndexL1[p]].i1)
                    {
                        Gizmos.DrawWireCube(bbOctree[i].center, (bbOctree[i].max - bbOctree[i].min));

                    }

                    if (i == pairIndexL1[collidablePairIndexL1[p]].i2)
                    {
                        Gizmos.DrawWireCube(bbOctree[i].center, (bbOctree[i].max - bbOctree[i].min));

                    }
                }
                

                Gizmos.color = Color.blue;
                for (int p = 0; p < collidablePairIndexL2.Count; p++)
                {
                    if (i == pairIndexL2[collidablePairIndexL2[p]].i1)
                    {
                        Gizmos.DrawWireCube(bbOctree[i].center, (bbOctree[i].max - bbOctree[i].min));

                    }

                    if (i == pairIndexL2[collidablePairIndexL2[p]].i2)
                    {
                        Gizmos.DrawWireCube(bbOctree[i].center, (bbOctree[i].max - bbOctree[i].min));

                    }
                }
            } 
        }
    }


    private void OnDestroy()
    {
        if (enabled)
        {
            positionsBuffer.Release();
            globalPositionsBuffer.Release();
            bbMinMaxBuffer.Release();
            rangeObjIndexBuffer.Release();
            bbOctreeBuffer.Release();
            collisionPairBuffer.Release();
            pairIndexL0Buffer.Release();
            pairIndexL1Buffer.Release();
            pairIndexL2Buffer.Release();
            collisionResultsL0Buffer.Release();
            collisionResultsL1Buffer.Release();
            collisionResultsL2Buffer.Release();
        }
    }
}
