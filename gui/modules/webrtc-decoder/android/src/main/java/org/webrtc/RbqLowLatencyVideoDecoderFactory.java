package org.webrtc;

import android.media.MediaCodec;
import android.media.MediaCodecInfo;
import android.media.MediaCrypto;
import android.media.MediaFormat;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.Surface;

import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;

import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.ByteBuffer;

public final class RbqLowLatencyVideoDecoderFactory implements VideoDecoderFactory {
    private static final String TAG = "RbqLowLatency";
    private final VideoDecoderFactory inner;

    public RbqLowLatencyVideoDecoderFactory(VideoDecoderFactory inner) {
        this.inner = inner;
    }

    @Nullable
    @Override
    public VideoDecoder createDecoder(VideoCodecInfo info) {
        final VideoDecoder decoder = inner.createDecoder(info);
        if (decoder instanceof AndroidVideoDecoder && Build.VERSION.SDK_INT >= 30) {
            try {
                final Field field = AndroidVideoDecoder.class.getDeclaredField("mediaCodecWrapperFactory");
                field.setAccessible(true);
                final MediaCodecWrapperFactory original = (MediaCodecWrapperFactory) field.get(decoder);
                field.set(decoder, (MediaCodecWrapperFactory) name -> new LowLatencyCodec(original.createByCodecName(name)));
            } catch (ReflectiveOperationException | RuntimeException e) {
                Log.w(TAG, "low-latency hook not installed, decoder unchanged: " + e);
            }
        }
        return decoder;
    }

    @Override
    public VideoCodecInfo[] getSupportedCodecs() {
        return inner.getSupportedCodecs();
    }

    @RequiresApi(30)
    private static final class LowLatencyCodec implements MediaCodecWrapper {
        private final MediaCodecWrapper codec;

        LowLatencyCodec(MediaCodecWrapper codec) {
            this.codec = codec;
        }

        @Override
        public void configure(MediaFormat format, Surface surface, MediaCrypto crypto, int flags) {
            final String mime = format.getString(MediaFormat.KEY_MIME);
            String name = "?";
            boolean supported = false;
            try {
                final MediaCodecInfo info = codec.getCodecInfo();
                name = info.getName();
                supported = info.getCapabilitiesForType(mime)
                                .isFeatureSupported(MediaCodecInfo.CodecCapabilities.FEATURE_LowLatency);
            } catch (RuntimeException e) {
                Log.w(TAG, "capability check failed: " + e);
            }
            if (supported) format.setInteger(MediaFormat.KEY_LOW_LATENCY, 1);
            Log.i(TAG, name + " " + mime + " low-latency " + (supported ? "on" : "not supported — unchanged"));
            codec.configure(format, surface, crypto, flags);
        }

        @Override public void start() { codec.start(); }
        @Override public void flush() { codec.flush(); }
        @Override public void stop() { codec.stop(); }
        @Override public void release() { codec.release(); }
        @Override public int dequeueInputBuffer(long timeoutUs) { return codec.dequeueInputBuffer(timeoutUs); }
        @Override public void queueInputBuffer(int index, int offset, int size, long presentationTimeUs, int flags) {
            codec.queueInputBuffer(index, offset, size, presentationTimeUs, flags);
        }
        @Override public int dequeueOutputBuffer(MediaCodec.BufferInfo info, long timeoutUs) {
            return codec.dequeueOutputBuffer(info, timeoutUs);
        }
        @Override public void releaseOutputBuffer(int index, boolean render) { codec.releaseOutputBuffer(index, render); }
        @Override public MediaFormat getInputFormat() { return codec.getInputFormat(); }
        @Override public MediaFormat getOutputFormat() { return codec.getOutputFormat(); }
        @Override public MediaFormat getOutputFormat(int index) { return codec.getOutputFormat(index); }
        @Override public ByteBuffer getInputBuffer(int index) { return codec.getInputBuffer(index); }
        @Override public ByteBuffer getOutputBuffer(int index) { return codec.getOutputBuffer(index); }
        @Override public Surface createInputSurface() { return codec.createInputSurface(); }
        @Override public void setParameters(Bundle params) { codec.setParameters(params); }
        @Override public MediaCodecInfo getCodecInfo() { return codec.getCodecInfo(); }
    }
}
