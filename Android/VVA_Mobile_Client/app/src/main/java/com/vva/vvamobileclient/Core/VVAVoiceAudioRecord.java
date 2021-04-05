package com.vva.vvamobileclient.Core;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

import android.media.AudioFormat;
import android.media.AudioRecord;
import android.media.MediaRecorder;
import android.os.Environment;
import android.util.Log;

import org.ros.exception.RosRuntimeException;

/**
 * Class in chanrge of recording a .wav file from the microphone
 */
public class VVAVoiceAudioRecord  {

        private static final int RECORDER_BPP = 16;
        private static final int RECORDER_WAV_FILE_CHANNELS = 1;
        private static final String AUDIO_RECORDER_FILE_EXT_WAV = ".wav";
        private static final String AUDIO_RECORDER_FOLDER = "vva_audio_record";
        private static final String AUDIO_RECORDER_TEMP_FILE = "record_temp.raw";
        private static final int RECORDER_SAMPLERATE = 16000;
        private static final int RECORDER_CHANNEL_CONFIG = AudioFormat.CHANNEL_IN_FRONT;
        private static final int RECORDER_AUDIO_ENCODING = AudioFormat.ENCODING_PCM_16BIT;

        private AudioRecord _recorder = null;
        private int _bufferSize = 0;
        private Thread _recordingThread = null;
        private boolean _isRecording = false;

        // Log tag String
        private static final String TAG = "VVAVoiceAudioRecord";


        /**
         * Constructor
         */
        public VVAVoiceAudioRecord() {
                _bufferSize = AudioRecord.getMinBufferSize(RECORDER_SAMPLERATE,
                        RECORDER_CHANNEL_CONFIG,
                        RECORDER_AUDIO_ENCODING);
        }

//        private void enableButton(int id,boolean isEnable){
//                ((Button)findViewById(id)).setEnabled(isEnable);
//        }

//        private void enableButtons(boolean isRecording) {
//                enableButton(R.id.btnStart,!isRecording);
//                enableButton(R.id.btnStop,isRecording);
//        }

        private String getFilename(){
                String filepath = Environment.getExternalStorageDirectory().getPath();
                File file = new File(filepath,AUDIO_RECORDER_FOLDER);

                if(!file.exists()){
                        file.mkdirs();
                }
                if(file.exists())
                        file.delete();

                return (file.getAbsolutePath() + "/vva_record" + AUDIO_RECORDER_FILE_EXT_WAV);
        }

        private String getTempFilename(){
                String filepath = Environment.getExternalStorageDirectory() .getPath();

                File file = new File(filepath, AUDIO_RECORDER_FOLDER);

                if(!file.exists()){
                        if (!file.mkdirs()) {
                                throw new RosRuntimeException("Could not create file path: " +
                                        file.getAbsolutePath());
                        }
                }

                File tempFile = new File(filepath, AUDIO_RECORDER_TEMP_FILE);

                if(tempFile.exists())
                        tempFile.delete();

                return (file.getAbsolutePath() + "/" + AUDIO_RECORDER_TEMP_FILE);
        }

        public void startRecording(){

                //Log.d(TAG, "Creating new AudioRecord...");

                _recorder = new AudioRecord(MediaRecorder.AudioSource.VOICE_RECOGNITION,
                        RECORDER_SAMPLERATE, RECORDER_CHANNEL_CONFIG, RECORDER_AUDIO_ENCODING, _bufferSize);

                int i = _recorder.getState();
                if(i==1) {
                        //Log.d(TAG, "AudioRecord created, starting recording...");
                        _recorder.startRecording();
                } else
                        throw new RosRuntimeException("Error on creating AudioRecord.");

                _isRecording = true;

                _recordingThread = new Thread(new Runnable() {
                        @Override
                        public void run() {
                                //Log.d(TAG, "New Thread started for writing audio data raw file");
                                writeAudioDataToFile();
                                //Log.d(TAG, "Finished Thread for writing audio data raw file");
                        }
                },"AudioRecorder Thread");

                _recordingThread.start();
        }

        private void writeAudioDataToFile(){
                byte data[] = new byte[_bufferSize];
                String filename = getTempFilename();
                FileOutputStream os = null;

                try {
                        os = new FileOutputStream(filename);
                } catch (FileNotFoundException e) {
                        e.printStackTrace();
                }

                int read = 0;

                if(null != os){
                        while(_isRecording){
                                read = _recorder.read(data, 0, _bufferSize);

                                if(AudioRecord.ERROR_INVALID_OPERATION != read){
                                        try {
                                                os.write(data);
                                        } catch (IOException e) {
                                                e.printStackTrace();
                                        }
                                }
                        }

                        try {
                                os.close();
                        } catch (IOException e) {
                                e.printStackTrace();
                        }
                }
        }

        public String stopRecording(){
                if(null != _recorder){
                        _isRecording = false;

                        int i = _recorder.getState();
                        if(i==1)
                                _recorder.stop();
                        _recorder.release();

                        _recorder = null;
                        _recordingThread = null;
                }

                String wav_filename = getFilename();

                copyWaveFile(getTempFilename(), wav_filename);
                deleteTempFile();

                return wav_filename;
        }

        private void deleteTempFile() {
                File file = new File(getTempFilename());

                file.delete();
        }

        private void copyWaveFile(String inFilename, String outFilename){
                FileInputStream in = null;
                FileOutputStream out = null;
                long totalAudioLen = 0;
                long totalDataLen = totalAudioLen + 36;
                long longSampleRate = RECORDER_SAMPLERATE;
                int channels = RECORDER_WAV_FILE_CHANNELS;
                long byteRate = RECORDER_BPP * RECORDER_SAMPLERATE * channels/8;

                byte[] data = new byte[_bufferSize];

                try {
                        in = new FileInputStream(inFilename);
                        out = new FileOutputStream(outFilename);
                        totalAudioLen = in.getChannel().size();
                        totalDataLen = totalAudioLen + 36;

                        //Log.d(TAG, "File size: " + totalDataLen);

                        WriteWaveFileHeader(out, totalAudioLen, totalDataLen,
                                longSampleRate, channels, byteRate);

                        while(in.read(data) != -1){
                                out.write(data);
                        }

                        in.close();
                        out.close();
                } catch (FileNotFoundException e) {
                        e.printStackTrace();
                } catch (IOException e) {
                        e.printStackTrace();
                }
        }

        private void WriteWaveFileHeader(
                FileOutputStream out, long totalAudioLen,
                long totalDataLen, long longSampleRate, int channels,
                long byteRate) throws IOException {

                byte[] header = new byte[44];

                header[0] = 'R'; // RIFF/WAVE header
                header[1] = 'I';
                header[2] = 'F';
                header[3] = 'F';
                header[4] = (byte) (totalDataLen & 0xff);
                header[5] = (byte) ((totalDataLen >> 8) & 0xff);
                header[6] = (byte) ((totalDataLen >> 16) & 0xff);
                header[7] = (byte) ((totalDataLen >> 24) & 0xff);
                header[8] = 'W';
                header[9] = 'A';
                header[10] = 'V';
                header[11] = 'E';
                header[12] = 'f'; // 'fmt ' chunk
                header[13] = 'm';
                header[14] = 't';
                header[15] = ' ';
                header[16] = 16; // 4 bytes: size of 'fmt ' chunk
                header[17] = 0;
                header[18] = 0;
                header[19] = 0;
                header[20] = 1; // format = 1
                header[21] = 0;
                header[22] = (byte) channels;
                header[23] = 0;
                header[24] = (byte) (longSampleRate & 0xff);
                header[25] = (byte) ((longSampleRate >> 8) & 0xff);
                header[26] = (byte) ((longSampleRate >> 16) & 0xff);
                header[27] = (byte) ((longSampleRate >> 24) & 0xff);
                header[28] = (byte) (byteRate & 0xff);
                header[29] = (byte) ((byteRate >> 8) & 0xff);
                header[30] = (byte) ((byteRate >> 16) & 0xff);
                header[31] = (byte) ((byteRate >> 24) & 0xff);
                header[32] = (byte) (2 * 16 / 8); // block align
                header[33] = 0;
                header[34] = RECORDER_BPP; // bits per sample
                header[35] = 0;
                header[36] = 'd';
                header[37] = 'a';
                header[38] = 't';
                header[39] = 'a';
                header[40] = (byte) (totalAudioLen & 0xff);
                header[41] = (byte) ((totalAudioLen >> 8) & 0xff);
                header[42] = (byte) ((totalAudioLen >> 16) & 0xff);
                header[43] = (byte) ((totalAudioLen >> 24) & 0xff);

                out.write(header, 0, 44);
        }

//        private View.OnClickListener btnClick = new View.OnClickListener() {
//                @Override
//                public void onClick(View v) {
//                        switch(v.getId()){
//                                case R.id.btnStart:{
//                                        AppLog.logString("Start Recording");
//
//                                        enableButtons(true);
//                                        startRecording();
//
//                                        break;
//                                }
//                                case R.id.btnStop:{
//                                        AppLog.logString("Start Recording");
//
//                                        enableButtons(false);
//                                        stopRecording();
//
//                                        break;
//                                }
//                        }
//                }
//        };
}