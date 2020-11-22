using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEditor;
using Functional;

[CustomEditor(typeof(PiecewiseCubicLine))]
public class LineInspector : Editor
{
	const float lineWidth = 4.0f;
	PiecewiseCubicLine line;

	//Global properties
	Transform handleTransform;
	Quaternion handleRotation;

	void initAll()
    {
		line = target as PiecewiseCubicLine;
		handleTransform = line.transform;
		handleRotation = Tools.pivotRotation == PivotRotation.Local ?
			handleTransform.rotation : Quaternion.identity;

		//Line Color
		Handles.color = Color.white;
	}

	private void OnSceneGUI()
	{
		initAll();

		HandleChangesToControlPoints();

		//Draw line between each point in world space
		Handles.DrawAAPolyLine(
			lineWidth, Returns<Vector3>.Map(
				line.GetRenderedPoints(),
				GetPointPosInWorld
			)
		);
	}

	public void HandleChangesToControlPoints()
    {
		//Handle transformations for each point (and save for view)
		int len = line.controlPoints.Length;

		for (int i = 0; i < len; ++i)
		{
			var p = GetPointPosInWorld(line.controlPoints[i]);
			CheckForAndApplyChangesToPoint(
				GetAndShowCurrentHandleForPoint(p),
				i
			);
		}
	}

	public Vector3 GetPointPosInWorld(Vector3 p)
    {
		return handleTransform.TransformPoint(p);
	}

	public Vector3 GetAndShowCurrentHandleForPoint(Vector3 pi)
    {
		return Handles.DoPositionHandle(pi, handleRotation);
	}

	public void CheckForAndApplyChangesToPoint(Vector3 pi, int index)
    {
		EditorGUI.BeginChangeCheck();
        pi = GetAndShowCurrentHandleForPoint(pi);

        //Apply changes to point
        if (EditorGUI.EndChangeCheck())
		{
			Undo.RecordObject(line, "Move Point");
			EditorUtility.SetDirty(line);
			pi = handleTransform.InverseTransformPoint(pi);
			line.controlPoints[index] = pi;
		}
	}
}
