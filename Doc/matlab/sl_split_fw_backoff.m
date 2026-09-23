function i_fw_safe = sl_split_fw_backoff(i_fw_cmd, iq, iq_ref, rpm)
%SL_SPLIT_FW_BACKOFF  Iq跟不上时减小弱磁，为q轴电压留余量。
c=foc.constants();
if i_fw_cmd<=1.0e-6
    i_fw_safe=0.0; return;
end
s=1.0; if rpm<0.0, s=-1.0; end
backoff=s*(iq-iq_ref)/max(c.FW_CURRENT_MAX,1.0e-6)*c.FW_BACKOFF;
backoff=min(max(backoff,0.0),1.0);
i_fw_safe=i_fw_cmd*(1.0-backoff);
end
