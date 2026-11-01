function ofdm_rx_pi_hw()
%#codegen
coder.cinclude('udp_linux.h');
coder.updateBuildInfo('addSourceFiles', 'udp_linux.c');

% definire arhitectură ofdm 
nfft = 64; cpLen = 16; nSym = 20; 
nullIdx  = [1:6 33 60:64]'; pilotIdx = [12 26 40 54]'; 
M = 64; numDataCarrs = nfft - length(nullIdx) - length(pilotIdx);

% setare așteptări buffer de recepție pentru 50 de cadre
cadre_dorite = 50; 
esantioane_pe_cadru = (nfft + cpLen) * (nSym + 3);
marja_esantioane = 100; 

% calcul memorie necesară pentru toate datele complexe convertite în bytes
expected_complex_samples = (cadre_dorite * esantioane_pe_cadru) + marja_esantioane; 
expected_bytes = expected_complex_samples * 8; 
No_frames = 50; 

% initializare array pentru stocare pachete
collected = zeros(expected_bytes, 1, 'uint8');
bytes_received = int32(0); 

% deschidere socket udp pentru ascultare
sock = int32(0); sock = coder.ceval('udp_init_rx'); 
buffer = zeros(1500, 1, 'uint8'); 
bytes_read = int32(0); empty_reads = int32(0);

fprintf(1, '[RX R4] Port 5005 deschis! Ascult pachetele...\n');

% buclă de recepție blocantă cu timeout
while bytes_received < expected_bytes && empty_reads < 30
    bytes_read = coder.ceval('udp_recv', sock, coder.wref(buffer), int32(1500));
    if bytes_read > 0
        % stocare date primite în bufferul mare
        collected(bytes_received+1 : bytes_received+bytes_read) = buffer(1:bytes_read);
        bytes_received = bytes_received + bytes_read;
        empty_reads = int32(0); 
    else
        empty_reads = empty_reads + 1; 
    end
end
% închidere port de rețea
coder.ceval('udp_close', sock);

% reconstrucție semnal complex din octeții primiți
payload_single = typecast(collected, 'single');
rx_complex = complex(double(payload_single(1:2:end)), double(payload_single(2:2:end)));

% secvențele standard pentru identificarea preambulului de sincronizare
qamSynch1=[0,0,0,0,0,0,0,1.4142,0,-1.4142,0,1.4142,0,-1.4142,0,-1.4142,0,-1.4142,0,1.4142,0,-1.4142,0,1.4142,0,-1.4142,0,-1.4142,0,-1.4142,0,-1.4142,0,1.4142,0,-1.4142,0,1.4142,0,1.4142,0,1.4142,0,-1.4142,0,1.4142,0,1.4142,0,1.4142,0,-1.4142,0,1.4142,0,1.4142,0,1.4142,0,0,0,0,0,0];
qamSynch2=[0,0,0,0,0,0,-1,-1,-1,-1,1,1,-1,-1,-1,1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,-1,-1,1,-1,0,1,-1,1,1,1,-1,1,1,1,-1,1,1,1,1,-1,1,-1,-1,-1,1,-1,1,-1,-1,-1,-1,0,0,0,0,0];
qamSynch1(end+1:length(qamSynch2))=0; qamSinch = [qamSynch1', qamSynch2'];

% trecerea preambulului în domeniul timp pentru comparație
preambul_ideal = ofdmmod(qamSinch, nfft, cpLen);
preambul_1 = preambul_ideal(:,1); 

tic; 

% căutarea punctului de start prin fereastra glisantă / auto-corelație
fereastra = rx_complex(1:min(200, length(rx_complex)));
corelatie = zeros(length(fereastra) - length(preambul_1), 1);

for i = 1:length(corelatie)
    corelatie(i) = abs(sum(fereastra(i : i+length(preambul_1)-1) .* conj(preambul_1)));
end

% determinare întârziere pe baza vârfului de corelație
[max_val, ~] = max(corelatie);
prag_sync = 0.3 * max_val; 
idx_valid = find(corelatie > prag_sync, 1, 'first');

delay_estimat = 0; 
if isempty(idx_valid)
    delay_estimat = 0; 
else
    delay_estimat = idx_valid(1) - 1; 
end

fprintf(1, '[RX R4] Sincronizare! Delay canal gasit: %d esantioane.\n', int32(delay_estimat));

% decupare semnal ignorând liniștea sau zgomotul inițial
start_idx = delay_estimat + 1;
lungime_cadru_complet = (nfft + cpLen) * (nSym + 3);

% calculăm câte cadre întregi s-au putut recupera
esantioane_disponibile = length(rx_complex) - start_idx + 1;
cadre_valide = floor(esantioane_disponibile / lungime_cadru_complet);

if cadre_valide > No_frames, cadre_valide = No_frames; end
if cadre_valide < 1, cadre_valide = 1; end 

end_idx = start_idx + cadre_valide * lungime_cadru_complet - 1;
rx_aliniat = rx_complex(start_idx : end_idx);

% trecem semnalul aliniat din domeniul timp inapoi în frecvență
rxDemod_raw = ofdmdemod(rx_aliniat, nfft, cpLen);
rxDemod_frames = reshape(rxDemod_raw, nfft, nSym + 3, cadre_valide);

% creăm o masca pentru indicii subpurtătoarelor care contin doar date utile
dataCarrIdx = zeros(numDataCarrs, 1);
idx = 1;
for k = 1:nfft
    is_null = false; for n = 1:length(nullIdx), if k == nullIdx(n), is_null = true; break; end; end
    is_pilot = false; for p = 1:length(pilotIdx), if k == pilotIdx(p), is_pilot = true; break; end; end
    if ~is_null && ~is_pilot
        dataCarrIdx(idx) = k;
        idx = idx + 1;
    end
end

% extragem eșantioanele ideale ale preambulului pentru estimarea canalului
preambul_ideal_activ = complex(zeros(numDataCarrs, 1));
for k = 1:numDataCarrs
    preambul_ideal_activ(k) = qamSinch(dataCarrIdx(k), 2);
end

% extragem eșantioanele ideale pentru piloți
preambul_ideal_pilots = complex(zeros(length(pilotIdx), 1));
for p = 1:length(pilotIdx)
    preambul_ideal_pilots(p) = qamSinch(pilotIdx(p), 2);
end

ref_pilots = [1; 1; 1; -1]; % secvența pilot teoretică

rx_data_only = complex(zeros(numDataCarrs, nSym, cadre_valide));

% bucla de egalizare si pilot tracking 
for c = 1:cadre_valide
    
    % calculăm răspunsul canalului exclusiv pe indicii piloților
    H_est_pilots = complex(zeros(length(pilotIdx), 1));
    for p = 1:length(pilotIdx)
        idx_p = pilotIdx(p);
        H_est_pilots(p) = rxDemod_frames(idx_p, 2, c) / preambul_ideal_pilots(p);
    end

    for sym = 1:nSym
        
        % determinam cat a deviat faza folosind piloții ca referință continuă
        phase_err_sum = 0;
        for p = 1:length(pilotIdx)
            idx_p = pilotIdx(p);
            pilot_receptionat = rxDemod_frames(idx_p, sym + 3, c) / H_est_pilots(p);
            phase_err_sum = phase_err_sum + angle(pilot_receptionat * conj(ref_pilots(p)));
        end
        avg_phase_err = phase_err_sum / length(pilotIdx); % media rotației de fază
        
        % aplicam corecția de canal și compensăm rotația fazei pe date
        for k = 1:numDataCarrs
            idx_fft = dataCarrIdx(k);
            H_est = rxDemod_frames(idx_fft, 2, c) / preambul_ideal_activ(k);
            date_brute = rxDemod_frames(idx_fft, sym + 3, c) / H_est;
            rx_data_only(k, sym, c) = date_brute * exp(-1i * avg_phase_err); % aplicare corecție faza
        end
        
    end
end

% calculăm evm recuantizând punctele pe cel mai apropiat simbol ideal
rxDemodQ = qamdemod(rx_data_only, M, 'UnitAveragePower', true);
rxRemodQ = qammod(rxDemodQ, M, 'UnitAveragePower', true);
err_rms = sqrt(mean(abs(rx_data_only(:) - rxRemodQ(:)).^2));

timp_demodulare = toc; 

fprintf(1, '[RX R4] Demodulare terminata in %.4f secunde.\n', timp_demodulare);

% stocare rezultate pe disk pentru extragere sftp către pc
fileID = fopen('/home/r4/rezultate_rx.txt', 'w');
if fileID ~= -1
    fprintf(fileID, '%f\n', err_rms);
    fprintf(fileID, '%f\n', double(bytes_received));
    fprintf(fileID, '%f\n', double(expected_bytes));
    fprintf(fileID, '%f\n', double(delay_estimat));
    fprintf(fileID, '%f\n', timp_demodulare);
    fprintf(fileID, '%f\n', double(cadre_valide));
    
    rx_real = real(rx_data_only(:)); rx_imag = imag(rx_data_only(:));
    fprintf(fileID, '%f\n', double(numel(rx_real)));
    for k = 1:numel(rx_real)
        fprintf(fileID, '%f %f\n', rx_real(k), rx_imag(k));
    end
    
    fprintf(fileID, '%f\n', double(numel(corelatie))); 
    for k = 1:numel(corelatie)
        fprintf(fileID, '%f\n', corelatie(k));
    end

    rxDemod_real = real(rxDemod_raw(:)); 
    rxDemod_imag = imag(rxDemod_raw(:));
    fprintf(fileID, '%f\n', double(numel(rxDemod_real)));
    for k = 1:numel(rxDemod_real)
        fprintf(fileID, '%f %f\n', rxDemod_real(k), rxDemod_imag(k));
    end
    
    fclose(fileID);
end
coder.ceval('system', ['chmod 777 /home/r4/rezultate_rx.txt', char(0)]);
end