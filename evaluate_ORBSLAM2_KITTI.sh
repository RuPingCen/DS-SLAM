#ORB for TUM rgb datasets

datapath="groundturth_euroc"


sequencename=(
"offset string" 
"00" 
"01" 
"02" 
"03" 
"04" 
"05" 
"06" 
"07" 
"08" 
"09" 
"10"
"11" 
"12" 
"13" 
)
configname=(
"offset string" 
"KITTI00-02.yaml" 
"KITTI00-02.yaml" 
"KITTI00-02.yaml" 
"KITTI03.yaml" 
"KITTI04-12.yaml" 
"KITTI04-12.yaml" 
"KITTI04-12.yaml" 
"KITTI04-12.yaml" 
"KITTI04-12.yaml" 
"KITTI04-12.yaml" 
"KITTI04-12.yaml" 
"KITTI04-12.yaml" 
"KITTI04-12.yaml" 
)


num=1
while [ $num -le 11 ]
do
	#循环运行5次
	times_k=3
	while [ $times_k -le 10 ]
	do
		sequence=${sequencename[num]}
		config=${configname[num]}
		echo ./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/${config} /media/crp/Samsun970CRP/dataSet/KITTI/sequences/${sequence}
		./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/${config} /media/crp/Samsun970CRP/dataSet/KITTI/sequences/${sequence}
		sleep 1
		mv CameraTrajectory.txt  result/ORBSLAM2_CameraTrajectory_KITTI_${sequence}_${times_k}.txt
		mv trackingtime.txt result/ORBSLAM2_trackingtime_KITTI_${sequence}_${times_k}.txt
		#evo_ape euroc gt est -va --plot --plot_mode xy --save_results out
		times_k=$(( $times_k + 1 ))
	done
 	num=$(( $num + 1 ))
	
done
  
