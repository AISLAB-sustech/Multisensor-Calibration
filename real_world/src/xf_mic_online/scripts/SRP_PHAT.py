#!/usr/bin/env python
#encoding:utf-8

import torch
import numpy as np
import time
# from scipy.io.wavfile import read
# from src.utils.bss import Duet
def doas2taus(doas, mics, fs, c=346.0):
    taus = (fs / c) * torch.matmul(doas, mics.transpose(0, 1))
    return taus
def steering(taus, n_fft):
    pi = 3.141592653589793
    frame_size = int((n_fft - 1) * 2)

    # Computing the different parts of the steering vector
    omegas = 2 * pi * torch.arange(0, n_fft) / frame_size
    omegas = omegas.repeat(taus.shape + (1,))
    taus = taus.unsqueeze(len(taus.shape)).repeat(
        (1,) * len(taus.shape) + (n_fft,)
    )

    # Assembling the steering vector
    a_re = torch.cos(-omegas * taus)
    a_im = torch.sin(-omegas * taus)
    a = torch.stack((a_re, a_im), len(a_re.shape))
    a = a.transpose(len(a.shape) - 3, len(a.shape) - 1).transpose(
        len(a.shape) - 3, len(a.shape) - 2
    )
    return a
def circle():
    resolution = 1 # degree
    pts = torch.zeros((int(360/resolution), 2), dtype=torch.float)
    discrete_angle = torch.FloatTensor(torch.arange(0.0,360.0,resolution)) / 180 * 3.141592654
    x = torch.cos(discrete_angle)
    y = torch.sin(discrete_angle)
    pts[:,0] = x
    pts[:, 1] = y
    return pts
class Covariance(torch.nn.Module):
    def __init__(self, average=True):
        super(Covariance, self).__init__()
        self.average = average
    def forward(self, Xs):
        XXs = self.cov(Xs=Xs, average=self.average)          # Xs : (1, ... , ...,2, channels)
        return XXs
    def cov(self, Xs, average=True):
        n_mics = Xs.shape[4]

        # the real and imaginary parts
        Xs_re = Xs[..., 0, :].unsqueeze(4)                          # (1, ... , ..., channels,1)
        Xs_im = Xs[..., 1, :].unsqueeze(4)

        # covariance
        Rxx_re = torch.matmul(Xs_re, Xs_re.transpose(3, 4)) + torch.matmul(
            Xs_im, Xs_im.transpose(3, 4)
        )

        Rxx_im = torch.matmul(Xs_re, Xs_im.transpose(3, 4)) - torch.matmul(
            Xs_im, Xs_re.transpose(3, 4)
        )

        #  the upper triangular part of the covariance matrices
        idx = torch.triu_indices(n_mics, n_mics)

        XXs_re = Rxx_re[..., idx[0], idx[1]]
        XXs_im = Rxx_im[..., idx[0], idx[1]]

        XXs = torch.stack((XXs_re, XXs_im), 3)                     # [1, frames, 201, 2, 21]

        if average is True:
            n_time_frames = XXs.shape[1]
            XXs = torch.mean(XXs, 1, keepdim=True)                  # [1,    1,   201, 2, 21]
            XXs = XXs.repeat(1, 2 , 1, 1, 1)             # CHANGE TO 2 [1, frames, 201, 2, 21]
        return XXs
class SrpPhat(torch.nn.Module):
    def __init__(
        self,
        mics,
        sample_rate=16000,
        speed_sound=343.0,
        eps=1e-20,
    ):
        super(SrpPhat,self).__init__()
        self.doas = circle()
        self.taus = doas2taus(self.doas, mics=mics, fs=sample_rate, c=speed_sound)   # (2562,6)
        # steering vector
        self.As   = steering(self.taus, 201)
        self.eps  = eps
    def forward(self, XXs):
        """
        (batch, time_steps, 3).
        XXs : tensor
            The covariance matrices of the input signal.
            (batch, time_steps, n_fft/2 + 1, 2, n_mics + n_pairs).
            [1, frames, 201, 2, 21]
        """
        #n_fft = XXs.shape[2]
        
        # steering vector
        # As = steering(self.taus, n_fft)    # torch.Size([2562, 201, 2, 6])

        doas = self.srp_phat(XXs=XXs, As=self.As, doas=self.doas, eps=self.eps)
        return doas
    def srp_phat(self, XXs, As, doas, eps=1e-20):
        """
        (batch, time_steps, 3)
        XXs : The covariance matrices of the input signal.
            (batch, time_steps, n_fft/2 + 1, 2, n_mics + n_pairs).
        As : steering vector of all the potential directions of arrival.
            (n_doas, n_fft/2 + 1, 2, n_mics).
        doas : All the possible directions
            (n_doas, 3).
        """
        n_mics = As.shape[3]

        # the pairs of microphones
        idx = torch.triu_indices(n_mics, n_mics) 

        # the demixing vector from the steering vector
        As_1_re = As[:, :, 0, idx[0, :]]            # torch.Size([2562, 201, 21])
        As_1_im = As[:, :, 1, idx[0, :]]
        As_2_re = As[:, :, 0, idx[1, :]]
        As_2_im = As[:, :, 1, idx[1, :]]
        Ws_re = As_1_re * As_2_re + As_1_im * As_2_im
        Ws_im = As_1_re * As_2_im - As_1_im * As_2_re
        Ws_re = Ws_re.reshape(Ws_re.shape[0], -1)
        Ws_im = Ws_im.reshape(Ws_im.shape[0], -1)


        # Get unique covariance values to reduce the number of computations
        XXs_val, XXs_idx = torch.unique(XXs, return_inverse=True, dim=1)

        # phase transform
        XXs_re = XXs_val[:, :, :, 0, :]
        XXs_im = XXs_val[:, :, :, 1, :]
        XXs_re = XXs_re.reshape((XXs_re.shape[0], XXs_re.shape[1], -1))
        XXs_im = XXs_im.reshape((XXs_im.shape[0], XXs_im.shape[1], -1))
        XXs_abs = torch.sqrt(XXs_re ** 2 + XXs_im ** 2) + eps
        XXs_re_norm = XXs_re / XXs_abs
        XXs_im_norm = XXs_im / XXs_abs

        # Project on the demixing vectors, and keep only real part
        Ys_A = torch.matmul(XXs_re_norm, Ws_re.transpose(0, 1))
        Ys_B = torch.matmul(XXs_im_norm, Ws_im.transpose(0, 1))
        Ys = Ys_A - Ys_B                                                   # torch.Size([1, 1, 10242])

        # Get maximum points
        _, doas_idx = torch.max(Ys, dim=2)
        # Repeat for each frame
        doas = (doas[doas_idx, :])[:, XXs_idx, :]
        return doas
class STFT(torch.nn.Module):
    """
    (batch, time, channels).
    """
    def __init__(
        self,
        sample_rate,
        win_length=25,
        hop_length=10,
        n_fft=400,
        window_fn=torch.hamming_window,
        normalized_stft=False,
        center=True,
        pad_mode="constant",
        onesided=True,
    ):
        super(STFT,self).__init__()
        self.sample_rate = sample_rate
        self.win_length = win_length
        self.hop_length = hop_length
        self.n_fft = n_fft
        self.normalized_stft = normalized_stft
        self.center = center
        self.pad_mode = pad_mode
        self.onesided = onesided

        # Convert win_length and hop_length from ms to samples
        self.win_length = int(
            round((self.sample_rate / 1000.0) * self.win_length)
        )
        self.hop_length = int(
            round((self.sample_rate / 1000.0) * self.hop_length)
        )
        self.window = window_fn(self.win_length)

    def forward(self, x):
        # multi-channel stft
        or_shape = x.shape                    # (1,sample data, channel)
        if len(or_shape) == 3:
            x = x.transpose(1, 2)             # (1,channel,sample data)
            x = x.reshape(or_shape[0] * or_shape[2], or_shape[1])  # (sample data, channel)
        stft = torch.stft(
            x,
            self.n_fft,
            self.hop_length,
            self.win_length,
            self.window,
            self.center,
            self.pad_mode,
            self.normalized_stft,
            self.onesided,
            return_complex=False,
        )                                  # (channels, ... , ...)
        # stft = torch.view_as_real(stft)
        # Retrieving the original dimensionality (batch,time, channels)
        if len(or_shape) == 3:
            stft = stft.reshape(
                or_shape[0],
                or_shape[2],
                stft.shape[1],
                stft.shape[2],
                stft.shape[3],
            )
            stft = stft.permute(0, 3, 2, 4, 1)
        else:
            # (batch, time, channels)
            stft = stft.transpose(2, 1)
        return stft # (1, ... , ...,2, channels)
