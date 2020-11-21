using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(PiecewiseCubicLine))]
public class LineInspector : Editor
{
	private const float lineWidth = 4.0f;

	private void OnSceneGUI()
	{
		PiecewiseCubicLine line = target as PiecewiseCubicLine;

		//Global properties
		Transform handleTransform = line.transform;
		Quaternion handleRotation = Tools.pivotRotation == PivotRotation.Local ?
			handleTransform.rotation : Quaternion.identity;

        //Line Color
        Handles.color = Color.white;

        //Handle transformations for each point (and save for view)
        int len = line.controlPoints.Length;
		Vector3[] pis = new Vector3[len];
		for (int i = 0; i < len; ++i)
        {
			Vector3 pi = handleTransform.TransformPoint(line.controlPoints[i]);

			Handles.DoPositionHandle(pi, handleRotation);

			EditorGUI.BeginChangeCheck();
			pi = Handles.DoPositionHandle(pi, handleRotation);

			//Apply changes to point
			if (EditorGUI.EndChangeCheck())
			{
				Undo.RecordObject(line, "Move Point");
				EditorUtility.SetDirty(line);
				line.controlPoints[i] = handleTransform.InverseTransformPoint(pi);
			}

			//Save for drawing
			pis[i] = pi;
		}

		//Draw line between each point
		Handles.DrawAAPolyLine(lineWidth, pis);
	}
}
