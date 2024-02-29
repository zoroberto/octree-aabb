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

public class MultiCPUMSM : MonoBehaviour
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

    [Header("Rendering")]
    public Shader renderingShader;
    public Color matColor;

    [Header("Debug Mode")]
    public bool debugModeLv0 = true; 
    public bool debugModeLv1 = true; 
    public bool debugModeLv2 = true; 

    [HideInInspector]
    private GameObject[] deformableObjectList;
    private CPUMSM[] deformableCPUMSM;
   
    // octree
    private OctreeData[] custOctree;
    private PairData[] pairIndexL0;
    private PairData[] pairIndexL1;
    private PairData[] pairIndexL2;
    private readonly int octree_size = 73;
    
    // collidable pair
    private readonly List<int> collidablePairIndexL0 = new List<int>();
    private readonly List<PairData> collidablePairIndexL1 = new List<PairData>();
    private readonly List<PairData> collidablePairIndexL2 = new List<PairData>();

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

    void Start()
    {
        SelectModelName();
        addDeformableObjectList();
        AddOctreePairIndex();
    }


    void addDeformableObjectList()
    {
        deformableObjectList = new GameObject[number_object];
        HashSet<Vector3> generatedPositions = new HashSet<Vector3>();
        deformableCPUMSM = new CPUMSM[number_object];
        custOctree = new OctreeData[number_object * octree_size];

        List<List<string>> csvData = ExporterAndImporter.ReadCSVFile(csv_file);
        Vector3 position = new Vector3();


        for (int i = 0; i < number_object; i++)
        {
            deformableObjectList[i] = new GameObject("Deformable Object " + i);
            deformableObjectList[i].transform.SetParent(this.transform);

            ////set position of the object 1). randomize 2).set the coord
            //Vector3 randomPosition;

            //do
            //{
            //    // Generate random position within the specified range
            //    float x = UnityEngine.Random.Range(rangeMin.x, rangeMax.x);
            //    float y = UnityEngine.Random.Range(rangeMin.y, rangeMax.y);
            //    float z = UnityEngine.Random.Range(rangeMin.z, rangeMax.z);
            //    randomPosition = new Vector3(x, y, z);
            //} while (generatedPositions.Contains(randomPosition));
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
                CPUMSM cpumsmScript = deformableObjectList[i].AddComponent<CPUMSM>();

                cpumsmScript.SetCoefficient(dt, gravity, speed);
                cpumsmScript.SetMeshData(modelName, useInteriorSpring);

                Shader tmp = Instantiate(renderingShader);
                Color randomColor = new Color(
                    UnityEngine.Random.value,
                    UnityEngine.Random.value,
                    UnityEngine.Random.value);

                //cpumsmScript.SetRenderer(tmp, matColor);
                cpumsmScript.SetRenderer(tmp, randomColor);
                cpumsmScript.SetCollidableObj(collidableObject);

                deformableCPUMSM[i] = cpumsmScript;
                deformableCPUMSM[i].StartObj();


            }
        }
    }


    int calculateStartIndex(int object_index, int level)
    {
        if (level == 1) return object_index * 73 + 1;
        if (level == 2) return object_index * 73 + 9;
        return object_index * 73 ;
    }

    int calculateEndIndex(int object_index, int level)
    {
        if (level == 1) return object_index * 73 + 8;
        if (level == 2) return object_index * 73 + 73;
        return object_index * 73 ;
    }

    private void AddOctreePairIndex()
    {
        // index pair
        List<PairIndex> indexPairsL0 = new List<PairIndex>();
        List<PairIndex> indexPairsL1 = new List<PairIndex>();
        List<PairIndex> indexPairsL2 = new List<PairIndex>();

        for(int i = 0; i < number_object; i++)
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


                for (int n = l0_st_idx; n<=l0_end_idx; n++)
                {
                    for (int m = j_l0_st_idx; m <= j_l0_end_idx; m++)
                    {
                        List<int> indices = new List<int>();
                        indices.Add(n);
                        indices.Add(m);
                        PairIndex pair = new PairIndex(indices);
                        indexPairsL0.Add(pair);
                    }
                }
                for (int n = l1_st_idx; n <= l1_end_idx; n++)
                {
                    for (int m = j_l1_st_idx; m <= j_l1_end_idx; m++)
                    {

                        List<int> indices = new List<int>();
                        indices.Add(n);
                        indices.Add(m);
                        PairIndex pair = new PairIndex(indices);
                        indexPairsL1.Add(pair);
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

    private void Update()
    {
        ImplementOctree();
        CheckCollisionOctreeLv0();
        CheckCollisionOctreeLv1();
        CheckCollisionOctreeLv2();
    }

    private void ImplementOctree() 
    {
        OctreeData[] custBB = new OctreeData[number_object];
        for (int i = 0; i < number_object; i++)
        {
            deformableCPUMSM[i].UpdateObj();
            Vector3[] position = deformableCPUMSM[i].GetPosition();

            CustMesh.Getvertices(position);
            custBB[i].min = CustMesh.minPos;
            custBB[i].max = CustMesh.maxPos;
            custBB[i].center = (custBB[i].max + custBB[i].min) / 2;
            custBB[i].size = custBB[i].max - custBB[i].min;

            deformableCPUMSM[i].SetBoundingBox(custBB[i].min, custBB[i].max);

            // Lv0, Initialize Lv0
            custOctree[i * octree_size].center = custBB[i].center; // center Lv0
            custOctree[i * octree_size].size = custBB[i].size; // size Lv0
            custOctree[i * octree_size].min = custBB[i].min; // min value Lv0
            custOctree[i * octree_size].max = custBB[i].max; // max value Lv0

            Vector3 centerOct = custOctree[i * octree_size].center;
            Vector3 sizeOct = custOctree[i * octree_size].size;

            // Lv2, Split to 8 children [0-7] 
            custOctree[i * octree_size + 1].center.x = centerOct.x - (sizeOct.x / 4);
            custOctree[i * octree_size + 1].center.y = centerOct.y + (sizeOct.y / 4);
            custOctree[i * octree_size + 1].center.z = centerOct.z - (sizeOct.z / 4);

            custOctree[i * octree_size + 2].center.x = centerOct.x + (sizeOct.x / 4);
            custOctree[i * octree_size + 2].center.y = centerOct.y + (sizeOct.y / 4);
            custOctree[i * octree_size + 2].center.z = centerOct.z - (sizeOct.z / 4);

            custOctree[i * octree_size + 3].center.x = centerOct.x - (sizeOct.x / 4);
            custOctree[i * octree_size + 3].center.y = centerOct.y - (sizeOct.y / 4);
            custOctree[i * octree_size + 3].center.z = centerOct.z - (sizeOct.z / 4);

            custOctree[i * octree_size + 4].center.x = centerOct.x + (sizeOct.x / 4);
            custOctree[i * octree_size + 4].center.y = centerOct.y - (sizeOct.y / 4);
            custOctree[i * octree_size + 4].center.z = centerOct.z - (sizeOct.z / 4);

            custOctree[i * octree_size + 5].center.x = centerOct.x - (sizeOct.x / 4);
            custOctree[i * octree_size + 5].center.y = centerOct.y + (sizeOct.y / 4);
            custOctree[i * octree_size + 5].center.z = centerOct.z + (sizeOct.z / 4);

            custOctree[i * octree_size + 6].center.x = centerOct.x + (sizeOct.x / 4);
            custOctree[i * octree_size + 6].center.y = centerOct.y + (sizeOct.y / 4);
            custOctree[i * octree_size + 6].center.z = centerOct.z + (sizeOct.z / 4);

            custOctree[i * octree_size + 7].center.x = centerOct.x - (sizeOct.x / 4);
            custOctree[i * octree_size + 7].center.y = centerOct.y - (sizeOct.y / 4);
            custOctree[i * octree_size + 7].center.z = centerOct.z + (sizeOct.z / 4);

            custOctree[i * octree_size + 8].center.x = centerOct.x + (sizeOct.x / 4);
            custOctree[i * octree_size + 8].center.y = centerOct.y - (sizeOct.y / 4);
            custOctree[i * octree_size + 8].center.z = centerOct.z + (sizeOct.z / 4);

            
            for (int j = 1; j <= 8; j++)
            {
                OctreeData oct = custOctree[i * octree_size + j];
                custOctree[i * octree_size + j].min = oct.Minimum(oct.center, (custBB[i].size / 4));
                custOctree[i * octree_size + j].max = oct.Maximum(oct.center, (custBB[i].size / 4));

                // Lv2, Split to 64 children

                custOctree[i * octree_size + j * 8 + 1].center.x = custOctree[i * octree_size + j].center.x - (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 1].center.y = custOctree[i * octree_size + j].center.y + (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 1].center.z = custOctree[i * octree_size + j].center.z - (sizeOct.z / 8);

                custOctree[i * octree_size + j * 8 + 2].center.x = custOctree[i * octree_size + j].center.x + (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 2].center.y = custOctree[i * octree_size + j].center.y + (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 2].center.z = custOctree[i * octree_size + j].center.z - (sizeOct.z / 8);

                custOctree[i * octree_size + j * 8 + 3].center.x = custOctree[i * octree_size + j].center.x - (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 3].center.y = custOctree[i * octree_size + j].center.y - (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 3].center.z = custOctree[i * octree_size + j].center.z - (sizeOct.z / 8);

                custOctree[i * octree_size + j * 8 + 4].center.x = custOctree[i * octree_size + j].center.x + (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 4].center.y = custOctree[i * octree_size + j].center.y - (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 4].center.z = custOctree[i * octree_size + j].center.z - (sizeOct.z / 8);

                custOctree[i * octree_size + j * 8 + 5].center.x = custOctree[i * octree_size + j].center.x - (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 5].center.y = custOctree[i * octree_size + j].center.y + (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 5].center.z = custOctree[i * octree_size + j].center.z + (sizeOct.z / 8);

                custOctree[i * octree_size + j * 8 + 6].center.x = custOctree[i * octree_size + j].center.x + (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 6].center.y = custOctree[i * octree_size + j].center.y + (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 6].center.z = custOctree[i * octree_size + j].center.z + (sizeOct.z / 8);

                custOctree[i * octree_size + j * 8 + 7].center.x = custOctree[i * octree_size + j].center.x - (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 7].center.y = custOctree[i * octree_size + j].center.y - (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 7].center.z = custOctree[i * octree_size + j].center.z + (sizeOct.z / 8);

                custOctree[i * octree_size + j * 8 + 8].center.x = custOctree[i * octree_size + j].center.x + (sizeOct.x / 8);
                custOctree[i * octree_size + j * 8 + 8].center.y = custOctree[i * octree_size + j].center.y - (sizeOct.y / 8);
                custOctree[i * octree_size + j * 8 + 8].center.z = custOctree[i * octree_size + j].center.z + (sizeOct.z / 8);

                for (int k = 1; k <= 8; k++)
                {
                    custOctree[i * octree_size + j * 8 + k].min = oct.Minimum(custOctree[i * octree_size + j * 8 + k].center, (custBB[i].size / 8));
                    custOctree[i * octree_size + j * 8 + k].max = oct.Maximum(custOctree[i * octree_size + j * 8 + k].center, (custBB[i].size / 8));

                }
            }
        }
    }

    private void CheckCollisionOctreeLv0()
    {
        collidablePairIndexL0.Clear();
        for (int i = 0; i < pairIndexL0.Length; i++)
        {
            if (Intersection.AABB(custOctree[pairIndexL0[i].i1].min, custOctree[pairIndexL0[i].i1].max,
            custOctree[pairIndexL0[i].i2].min, custOctree[pairIndexL0[i].i2].max))
            {
                if (debugModeLv0)
                {
                    if (!collidablePairIndexL0.Contains(pairIndexL0[i].i1))
                        collidablePairIndexL0.Add(pairIndexL0[i].i1);

                    if (!collidablePairIndexL0.Contains(pairIndexL0[i].i2))
                        collidablePairIndexL0.Add(pairIndexL0[i].i2);
                }
            }
        }
    }

    private void CheckCollisionOctreeLv1()
    {
        collidablePairIndexL1.Clear();
        for (int i = 0; i < pairIndexL1.Length; i++)
        {
            int l0_index_obj1 = (int)Mathf.Floor(pairIndexL1[i].i1 / octree_size) * octree_size;
            int l0_index_obj2 = (int)Mathf.Floor(pairIndexL1[i].i2 / octree_size) * octree_size;

            if (Intersection.AABB(custOctree[l0_index_obj1].min, custOctree[l0_index_obj1].max,
                custOctree[l0_index_obj2].min, custOctree[l0_index_obj2].max))
            {
                if (Intersection.AABB(custOctree[pairIndexL1[i].i1].min, custOctree[pairIndexL1[i].i1].max,
                    custOctree[pairIndexL1[i].i2].min, custOctree[pairIndexL1[i].i2].max))
                {
                    if (debugModeLv1)
                    {
                        // use any to map multi pair and not duplicte of Lv1
                        if (!collidablePairIndexL1.Any(c => c.Equals(pairIndexL1[i])))
                            collidablePairIndexL1.Add(pairIndexL1[i]);
                    }



                }
            }
        }
    }

    private int calcIndexObjectLevel0(int object_index)
    {
        int level = (int)Mathf.Floor(object_index / octree_size);


        //if (level == 1) return level * 73;
        if (level != 0) return level * octree_size;
        return (int)Mathf.Floor(object_index / octree_size);
    }

    private int calcIndexObjectLevel1(int object_index )
    {
        int level = (int)Mathf.Floor(object_index / octree_size);

        if (level == 1) return (int)Mathf.Floor((object_index - (1 + level)) / 8) + level * 64;
        if (level == 2) return (int)Mathf.Floor((object_index - (1 + level)) / 8) + level * 64;
        return (int)Mathf.Floor((object_index - 1) / 8) + level * 64 ;
    }

    private void CheckCollisionOctreeLv2()
    {

        collidablePairIndexL2.Clear();
        for (int i = 0; i < pairIndexL2.Length; i++)
        {
            int l0_index_obj1 = calcIndexObjectLevel0(pairIndexL2[i].i1);
            int l0_index_obj2 = calcIndexObjectLevel0(pairIndexL2[i].i2);

            int l1_index_obj1 = calcIndexObjectLevel1(pairIndexL2[i].i1);
            int l1_index_obj2 = calcIndexObjectLevel1(pairIndexL2[i].i2);

            if (Intersection.AABB(custOctree[l0_index_obj1].min, custOctree[l0_index_obj1].max,
                custOctree[l0_index_obj2].min, custOctree[l0_index_obj2].max))
            {
                if (Intersection.AABB(custOctree[l1_index_obj1].min, custOctree[l1_index_obj1].max,
                    custOctree[l1_index_obj2].min, custOctree[l1_index_obj2].max))
                {
                    if (Intersection.AABB(custOctree[pairIndexL2[i].i1].min, custOctree[pairIndexL2[i].i1].max,
                        custOctree[pairIndexL2[i].i2].min, custOctree[pairIndexL2[i].i2].max))
                    {
                        if (debugModeLv2)
                        {
                            if (!collidablePairIndexL2.Any(c => c.Equals(pairIndexL2[i])))
                                collidablePairIndexL2.Add(pairIndexL2[i]);
                        }
                    }

                }
            }
        }

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
        if (custOctree != null )
        {
            for (int i = 0; i < custOctree.Length; i++)
            {
                if (debugModeLv0)
                {
                    Gizmos.color = Color.red;
                    for (int j = 0; j < collidablePairIndexL0.Count; j++)
                    {
                        if (i == collidablePairIndexL0[j])
                        {
                            Gizmos.DrawWireCube(custOctree[collidablePairIndexL0[j]].center, custOctree[collidablePairIndexL0[j]].size);
                        }
                    }
                }

                if (debugModeLv1)
                {
                    Gizmos.color = Color.green;
                    for (int j = 0; j < collidablePairIndexL1.Count; j++)
                    {
                        
                        if (i == collidablePairIndexL1[j].i1)
                        {
                            Vector3 size = custOctree[collidablePairIndexL1[j].i1].max - custOctree[collidablePairIndexL1[j].i1].min;
                            Gizmos.DrawWireCube(custOctree[collidablePairIndexL1[j].i1].center, size);
                        }
                        if (i == collidablePairIndexL1[j].i2)
                        {
                            Vector3 size = custOctree[collidablePairIndexL1[j].i2].max - custOctree[collidablePairIndexL1[j].i2].min;
                            Gizmos.DrawWireCube(custOctree[collidablePairIndexL1[j].i2].center, size);
                        }
                    }
                }

                if (debugModeLv2)
                {
                    Gizmos.color = Color.blue;
                    for (int j = 0; j < collidablePairIndexL2.Count; j++)
                    {
                       
                        if (i == collidablePairIndexL2[j].i1)
                        {
                            Vector3 size = custOctree[collidablePairIndexL2[j].i1].max - custOctree[collidablePairIndexL2[j].i1].min;
                            Gizmos.DrawWireCube(custOctree[collidablePairIndexL2[j].i1].center, size);
                        }
                        if (i == collidablePairIndexL2[j].i2)
                        {
                            Vector3 size = custOctree[collidablePairIndexL2[j].i2].max - custOctree[collidablePairIndexL2[j].i2].min;
                            Gizmos.DrawWireCube(custOctree[collidablePairIndexL2[j].i2].center, size);
                        }
                    }
                }
            }
        }
    }

}
