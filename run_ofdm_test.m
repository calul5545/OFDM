clc; clear all; close all;
% ștergere cache pc și plăci
cale_script = fileparts(mfilename('fullpath'));
if ~isempty(cale_script), cd(cale_script); end

% configurare rețea și adrese ip pentru procesul de ssh spre plăci
ip_t8_wifi = '172.20.10.5'; user_t8 = 't8'; parola_t8 = 'A24cdfb78'; ip_t8_eth = '10.0.0.1';
ip_r4_wifi = '172.20.10.4'; user_r4 = 'r4'; parola_r4 = 'A24cdfb78'; ip_r4_eth = '10.0.0.2';

disp('0. Ștergere cache pc și plăci...');
pi_tx = raspi(ip_t8_wifi, user_t8, parola_t8);
pi_rx = raspi(ip_r4_wifi, user_r4, parola_r4);

% eliberare memorie linux de rulari vechi blocate
system(pi_rx, 'sudo killall -9 ofdm_rx_pi_hw 2>/dev/null || true');
system(pi_tx, 'sudo killall -9 ofdm_tx_pi_hw 2>/dev/null || true');

disp('1. Compilare și deploy ...');
% compilare cod pentru arhitectura arm a plăcii rx
hw_rx = coder.hardware('Raspberry Pi'); hw_rx.DeviceAddress = ip_r4_wifi; 
hw_rx.Username = user_r4; hw_rx.Password = parola_r4;
cfg_rx = coder.config('exe'); cfg_rx.Hardware = hw_rx; cfg_rx.GenerateExampleMain = 'GenerateCodeAndCompile'; 
codegen -config cfg_rx ofdm_rx_pi_hw

% compilare cod pentru arhitectura arm a plăcii tx
hw_tx = coder.hardware('Raspberry Pi'); hw_tx.DeviceAddress = ip_t8_wifi; 
hw_tx.Username = user_t8; hw_tx.Password = parola_t8;
cfg_tx = coder.config('exe'); cfg_tx.Hardware = hw_tx; cfg_tx.GenerateExampleMain = 'GenerateCodeAndCompile'; 
codegen -config cfg_tx ofdm_tx_pi_hw

disp('2. Execuție transmisie hardware ...');
cale_rx = strtrim(system(pi_rx, 'find /home/r4/MATLAB_ws -type f -name "ofdm_rx_pi_hw.elf" | head -n 1'));
cale_tx = strtrim(system(pi_tx, 'find /home/t8/MATLAB_ws -type f -name "ofdm_tx_pi_hw.elf" | head -n 1'));

% rulare receptor în background
system(pi_rx, ['sudo nohup ' cale_rx ' > /home/r4/log_rx.txt 2>&1 &']);
pause(2);

% declanșare script emisie
log_tx = system(pi_tx, ['sudo ' cale_tx]);

disp('Așteptăm procesarea semnalului (10 sec) ...');
pause(10);

try
    disp('3. Descărcare rezultate si generare grafice ...');
    
    % recreere model teoretic de referință local pe pc
    M_head = 2; M = 64; nSym = 20; nfft = 64; cpLen = 16; No_frames = 50;
    nullIdx  = [1:6 33 60:64]'; pilotIdx = [12 26 40 54]'; pilots = repmat([1;1;1;-1],1,nSym+1);
    numDataCarrs = nfft - length(nullIdx) - length(pilotIdx);

    inHead = mod(0:numDataCarrs-1, M_head)';
    qamHead = qammod(inHead, M_head, 'UnitAveragePower', true);
    
    inData = mod(0:(numDataCarrs*nSym)-1, M)';
    inData = reshape(inData, numDataCarrs, nSym);
    tx_qam_data = qammod(inData, M, 'UnitAveragePower', true);
    
    qamSym = [qamHead, tx_qam_data];
    semnal_timp_tx = ofdmmod(qamSym, nfft, cpLen, nullIdx, pilotIdx, pilots);
    
    % modelare ecouri pentru desenarea pe grafice
    delay_esantioane = 15; 
    h_canal = complex(zeros(20, 1)); 
    h_canal(delay_esantioane + 1) = 1;        
    h_canal(end) = 0.4 - 0.2i;  
    semnal_timp_rx = filter(h_canal, 1, semnal_timp_tx(:));
    
    
    semnal_timp_rx = awgn(semnal_timp_rx, 30, 'measured');
    
    padding = complex(zeros(100, 1));
    semnal_timp_tx = [semnal_timp_tx(:); padding];
    semnal_timp_rx = [semnal_timp_rx(:); padding];

    % interpretare jurnale sistem tx
    t_mod_estimat = 0;
    tokens_tx = regexp(log_tx, 'Modulare completă în ([\d\.]+) secunde', 'tokens');
    if ~isempty(tokens_tx), t_mod_estimat = str2double(tokens_tx{1}{1}); end

    % descărcare valori decodate calculate direct pe hardware
    getFile(pi_rx, '/home/r4/rezultate_rx.txt', pwd);
    fileID = fopen('rezultate_rx.txt', 'r');
    
    if fileID ~= -1
        err_rms      = fscanf(fileID, '%f', 1);
        b_recv       = fscanf(fileID, '%f', 1);
        b_exp        = fscanf(fileID, '%f', 1);
        delay_est    = fscanf(fileID, '%f', 1);
        t_demod      = fscanf(fileID, '%f', 1);
        cadre_valide = fscanf(fileID, '%f', 1);
        
        n_elem_rx = fscanf(fileID, '%f', 1);
        date_rx = fscanf(fileID, '%f %f', 2 * n_elem_rx);
        rx_complex = complex(date_rx(1:2:end), date_rx(2:2:end));
        
        n_elem_cor = fscanf(fileID, '%f', 1);
        corelatie = fscanf(fileID, '%f', n_elem_cor);
        
        n_elem_demod = fscanf(fileID, '%f', 1);
        date_demod = fscanf(fileID, '%f %f', 2 * n_elem_demod);
        rxDemod_raw = complex(date_demod(1:2:end), date_demod(2:2:end));
        
        fclose(fileID);
        
        % diagnostic al performanței și comparație pc vs hardware
        tx_ideal_liniar = repmat(tx_qam_data(:), cadre_valide, 1);
        rx_receptie_liniar = rx_complex(:);

        true_err_rms = sqrt(mean(abs(rx_receptie_liniar - tx_ideal_liniar).^2));

        rx_intregi = qamdemod(rx_receptie_liniar, M, 'UnitAveragePower', true);
        tx_intregi = repmat(inData(:), cadre_valide, 1);

        erori_simbol = sum(rx_intregi ~= tx_intregi);
        ser = erori_simbol / numel(tx_intregi);

        tx_biti = int2bit(tx_intregi, log2(M));
        rx_biti = int2bit(rx_intregi, log2(M));
        [~, ber] = biterr(tx_biti, rx_biti);
        
        rxDemod_frames_pc = reshape(rxDemod_raw, nfft, nSym + 3, cadre_valide);
        
        % extragere date neegalizate pentru figură
        dataCarrIdx_pc = zeros(numDataCarrs, 1);
        idx_pc = 1;
        for k = 1:nfft
            is_null = false; for n = 1:length(nullIdx), if k == nullIdx(n), is_null = true; break; end; end
            is_pilot = false; for p = 1:length(pilotIdx), if k == pilotIdx(p), is_pilot = true; break; end; end
            if ~is_null && ~is_pilot
                dataCarrIdx_pc(idx_pc) = k;
                idx_pc = idx_pc + 1;
            end
        end
        
        rx_data_neegalizat = complex(zeros(numDataCarrs, nSym, cadre_valide));
        for c = 1:cadre_valide
            for sym = 1:nSym
                for k = 1:numDataCarrs
                    idx_fft = dataCarrIdx_pc(k);
                    rx_data_neegalizat(k, sym, c) = rxDemod_frames_pc(idx_fft, sym + 3, c);
                end
            end
        end
        
        % calcul de eficiență a algoritmilor în microsecunde
        simboluri_per_cadru = nSym + 3;
        total_simboluri_tx = No_frames * simboluri_per_cadru; 
        total_simboluri_rx = cadre_valide * simboluri_per_cadru;
        
        timp_mod_per_simbol = (t_mod_estimat / total_simboluri_tx) * 1e6;
        timp_demod_per_simbol = (t_demod / total_simboluri_rx) * 1e6;
     
        % printare rezultate
        disp(' '); 
        disp('==================== STATISTICI SISTEM ====================');
        disp(['Cadre OFDM procesate:   ', num2str(cadre_valide)]);
        disp(['Delay canal măsurat:    ', num2str(delay_est), ' esantioane']);
        disp(['Timp Modulare / Simbol: ', num2str(timp_mod_per_simbol, '%6.2f'), ' microsecunde']);
        disp(['Timp Demodul. / Simbol: ', num2str(timp_demod_per_simbol, '%6.2f'), ' microsecunde']);
        disp('================= DIAGNOSTIC =================');
        disp(['Error Vector Magnitude: ', num2str(true_err_rms*100, '%6.2f'), ' %']);
        disp(['Root Mean Square:       ', num2str(true_err_rms, '%6.4f')]);
        disp(['SER:                    ', num2str(ser*100, '%6.2f'), ' %']);
        disp(['BER:                    ', num2str(ber*100, '%6.2f'), ' %']);
               
        % desenare grafice de analiză
        figure('Name', 'Analiză canal de comunicație', 'Color', 'w', 'Position', [50, 100, 1000, 400]);
        subplot(1, 2, 1);
        stem(0:length(h_canal)-1, abs(h_canal), 'filled', 'b', 'LineWidth', 1.5);
        title('Răspunsul la impuls al canalului fizic');
        xlabel('Eșantioane (întarziere)'); ylabel('Amplitudine'); grid on;
        
        subplot(1, 2, 2);
        esantioane_viz = 1:min(200, length(semnal_timp_rx)); 
        plot(esantioane_viz, real(semnal_timp_tx(esantioane_viz)), 'b', 'LineWidth', 1.5); hold on;
        plot(esantioane_viz, real(semnal_timp_rx(esantioane_viz + delay_esantioane)), 'r', 'LineWidth', 1.2);
        title('Date ofdm traversând canalul');
        xlabel('Timp (eșantioane)'); ylabel('Amplitudine (partea reală)');
        legend('Semnal emis (tx)', 'Semnal recepție aliniat (rx)'); grid on;
        
        figure('Name', 'Auto-corelație', 'Color', 'w', 'Position', [100, 150, 800, 300]);
        plot(corelatie, 'g', 'LineWidth', 1.5); hold on;
        xline(delay_est + 1, 'r--', 'LineWidth', 1.5, 'Label', ['Delay gasit: ' num2str(delay_est)]);
        title('Algoritm de auto-corelație pentru găsirea preambulului');
        xlabel('Eșantioane'); ylabel('Valoare corelație'); grid on;
        
        figure('Name', 'Constelații OFDM', 'Color', 'w', 'Position', [150, 200, 800, 400]);
        subplot(1, 2, 1);
        plot(tx_qam_data(:), 'r.', 'MarkerSize', 6);
        title('Date modulate (tx)');
        xlabel('In-fază'); ylabel('Quadratură'); grid on; axis square; xlim([-1.5 1.5]); ylim([-1.5 1.5]);
        
        subplot(1, 2, 2);
        scatter(real(rx_complex), imag(rx_complex), 15, 'g', 'filled', 'MarkerFaceAlpha', 0.03);
        title(['Date demodulate (rx) | Evm: ' num2str(true_err_rms*100, '%.1f') '%']);
        xlabel('In-fază'); ylabel('Quadratură'); grid on; axis square; xlim([-1.5 1.5]); ylim([-1.5 1.5]);

        h_canal_frecventa = fft(h_canal, nfft);
        frecvente = linspace(-nfft/2, nfft/2 - 1, nfft);

        figure('Name', 'Răspunsul canalului în frecvență', 'Color', 'w', 'Position', [200, 250, 800, 400]);
        subplot(1, 2, 1);
        plot(frecvente, fftshift(10*log10(abs(h_canal_frecventa))), 'b', 'LineWidth', 1.5);
        title('Atenuarea canalului (amplitudine)');
        xlabel('Subpurtătoare (index frecventa)'); ylabel('Magnitudine (db)'); grid on;

        subplot(1, 2, 2);
        plot(frecvente, fftshift(angle(h_canal_frecventa)), 'r', 'LineWidth', 1.5);
        title('Distorsiunea de fază a canalului');
        xlabel('Subpurtătoare (index frecventă)'); ylabel('Faza (radiani)'); grid on;

        figure('Name', 'Densitatea spectrala de putere (PSD)', 'Color', 'w', 'Position', [250, 300, 800, 400]);
        pwelch(semnal_timp_tx, [], [], nfft*4, 'centered'); hold on;
        pwelch(semnal_timp_rx, [], [], nfft*4, 'centered');
        title('Spectrul semnalului ofdm (tx vs rx)');
        legend('Semnal emis (tx)', 'Semnal recepționat (cu zgomot)');

        figure('Name', 'Efectul Pilot Tracking-ului', 'Color', 'w', 'Position', [300, 350, 800, 400]);
        
        subplot(1, 2, 1);
        scatter(real(rx_data_neegalizat(:)), imag(rx_data_neegalizat(:)), 15, 'r', 'filled', 'MarkerFaceAlpha', 0.03);
        title('Constelație brută (fară corecție Piloți)');
        xlabel('In-fază'); ylabel('Quadratură'); grid on; axis square;
        
        subplot(1, 2, 2);
        scatter(real(rx_complex), imag(rx_complex), 15, 'g', 'filled', 'MarkerFaceAlpha', 0.03);
        title('Constelația corectată (h\_est + Piloti)');
        xlabel('In-fază'); ylabel('Quadratură'); grid on; axis square; 
        xlim([-1.5 1.5]); ylim([-1.5 1.5]);

    end
catch ME
    disp('Eroare la extragerea sau analiza datelor.');
    disp(ME.message);
end