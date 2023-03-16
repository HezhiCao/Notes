# PLY Format

## Header
* start with `ply` and end with `end_header`
* each element uses the format of `element name element_number`, followed by several `property datatype property_name` specify the data type and order of properties for this element.
    - `property list`: e.g. `property list uchar int vertex_index`: the property `vertex_index` contains first an unsigned char telling how many indices the property contains, followed by a list of indexes.
    - list of the scalar data types a property may have:
    |name       |type        |number of bytes|
    |-----------|----------------------------|
    |char       |character                 |1|
    |uchar      |unsigned character        |1|
    |short      |short integer             |2|
    |ushort     |unsigned short integer    |2|
    |int        |integer                   |4|
    |uint       |unsigned integer          |4|
    |float      |single-precision float    |4|
    |double     |double-precision float    |8|

## Body
* Managed by the order and datatype described by the `property`.
```
ply
format ascii 1.0  
comment hints information
element vertex 8
property float x
property float y
property float z
element face 7
property list uchar int vertex_index
element edge 5
property int vertex1
property int vertex2
end_header
0 0 0
0 0 1
0 1 1
0 1 0
1 0 0
1 0 1
1 1 0
1 1 1
3 0 1 2
3 0 2 3
4 7 6 5 4
4 0 4 5 1
4 1 5 6 2
4 2 6 7 3
4 3 7 4 0
0 1
1 2
2 3
3 0
2 0
```
